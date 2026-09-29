/*
 * TTML subtitle decoder
 * Copyright (c) 2026 Dalet
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * TTML subtitle decoder, converting paragraph payloads to ASS.
 *
 * Packets are the inner XML of a TTML <p> element, optionally wrapped in a
 * <span style=".." region=".."> element (the format produced by the ttml
 * encoder and the ttml demuxer). Styles and regions are looked up in the
 * <head> stored in the extradata in the format used by the ttml muxer.
 *
 * Supported: bold, italic, underline, colour, background colour (as ASS
 * outline colour), font family, text/display alignment, <br> and nested
 * <span> inheritance.
 * @see https://www.w3.org/TR/ttml2/
 */

#include <libxml/parser.h>
#include <libxml/tree.h>

#include "libavutil/bprint.h"
#include "libavutil/mem.h"
#include "libavutil/parseutils.h"

#include "avcodec.h"
#include "ass.h"
#include "codec_internal.h"
#include "ttmlenc.h"

#define TTML_NS_TTS "http://www.w3.org/ns/ttml#styling"
#define MAX_DEPTH   64

typedef struct TTMLStyle {
    int64_t color;          // 0xAARRGGBB, -1 if unset
    int64_t bg;             // 0xAARRGGBB, -1 if unset
    int bold, italic, underline;    // -1 if unset
    int text_align;         // 1 left, 2 center, 3 right, -1 unset
    int display_align;      // 0 before, 1 center, 2 after, -1 unset
    char font[64];
} TTMLStyle;

typedef struct TTMLContext {
    FFASSDecoderContext ass;    // must be first
    xmlDocPtr head_doc;
    xmlNodePtr head;
    char *prefix;               // "<tt ...>" used to wrap every payload
} TTMLContext;

typedef struct TTMLWalk {
    TTMLContext *s;
    AVBPrint *bp;
    int an;
    int space;                  // last output char was whitespace
} TTMLWalk;

static const TTMLStyle ttml_style_unset = {
    .color = -1, .bg = -1, .bold = -1, .italic = -1, .underline = -1,
    .text_align = -1, .display_align = -1,
};

static int ttml_is_elem(xmlNodePtr n, const char *name)
{
    return n->type == XML_ELEMENT_NODE && !strcmp((const char *)n->name, name);
}

static xmlNodePtr ttml_find_by_id(xmlNodePtr parent, const char *name,
                                  const char *id, int depth)
{
    xmlNodePtr n;

    if (!parent || depth > MAX_DEPTH)
        return NULL;
    for (n = parent->children; n; n = n->next) {
        xmlNodePtr r;
        if (ttml_is_elem(n, name)) {
            xmlChar *v = xmlGetNsProp(n, (const xmlChar *)"id", XML_XML_NAMESPACE);
            int match = v && !strcmp((const char *)v, id);
            xmlFree(v);
            if (match)
                return n;
        }
        if ((r = ttml_find_by_id(n, name, id, depth + 1)))
            return r;
    }
    return NULL;
}

static int64_t ttml_parse_color(const char *str)
{
    uint8_t rgba[4];

    if (!strcmp(str, "transparent"))
        return 0;
    if (av_parse_color(rgba, str, -1, NULL) < 0)
        return -1;
    return (int64_t)rgba[3] << 24 | rgba[0] << 16 | rgba[1] << 8 | rgba[2];
}

static void ttml_set_attr(TTMLStyle *st, const char *name, const char *v)
{
    if (!strcmp(name, "color")) {
        st->color = ttml_parse_color(v);
    } else if (!strcmp(name, "backgroundColor")) {
        st->bg = ttml_parse_color(v);
    } else if (!strcmp(name, "fontWeight")) {
        st->bold = !strcmp(v, "bold");
    } else if (!strcmp(name, "fontStyle")) {
        st->italic = !strcmp(v, "italic") || !strcmp(v, "oblique");
    } else if (!strcmp(name, "textDecoration")) {
        st->underline = !!strstr(v, "underline") && !strstr(v, "noUnderline");
    } else if (!strcmp(name, "fontFamily")) {
        // first family only, without quotes
        size_t n = strcspn(v, ",");
        while (*v == ' ' || *v == '\'' || *v == '"')
            v++, n--;
        while (n && (v[n - 1] == ' ' || v[n - 1] == '\'' || v[n - 1] == '"'))
            n--;
        if (n && n < sizeof(st->font) &&
            strncmp(v, "default", n) && strncmp(v, "monospace", n) &&
            strncmp(v, "sansSerif", n) && strncmp(v, "serif", n)) {
            memcpy(st->font, v, n);
            st->font[n] = 0;
        } else {
            st->font[0] = 0;
        }
    } else if (!strcmp(name, "textAlign")) {
        if (!strcmp(v, "left") || !strcmp(v, "start"))
            st->text_align = 1;
        else if (!strcmp(v, "center"))
            st->text_align = 2;
        else if (!strcmp(v, "right") || !strcmp(v, "end"))
            st->text_align = 3;
    } else if (!strcmp(name, "displayAlign")) {
        if (!strcmp(v, "before"))
            st->display_align = 0;
        else if (!strcmp(v, "center"))
            st->display_align = 1;
        else if (!strcmp(v, "after"))
            st->display_align = 2;
    }
}

static void ttml_apply_inline(xmlNodePtr n, TTMLStyle *st)
{
    xmlAttrPtr a;

    for (a = n->properties; a; a = a->next) {
        xmlChar *v;
        if (!a->ns || !a->ns->href ||
            strcmp((const char *)a->ns->href, TTML_NS_TTS))
            continue;
        v = xmlNodeGetContent((xmlNodePtr)a);
        if (v)
            ttml_set_attr(st, (const char *)a->name, (const char *)v);
        xmlFree(v);
    }
}

/* Apply the styles referenced by n's style attribute, then n's own tts:*. */
static void ttml_apply_element(TTMLContext *s, xmlNodePtr n, TTMLStyle *st,
                               int depth)
{
    xmlChar *refs;
    char *dup, *tok, *save = NULL;

    if (depth > 8)
        return;
    if ((refs = xmlGetNoNsProp(n, (const xmlChar *)"style"))) {
        dup = av_strdup((const char *)refs);
        xmlFree(refs);
        for (tok = dup ? av_strtok(dup, " \t\r\n", &save) : NULL; tok;
             tok = av_strtok(NULL, " \t\r\n", &save)) {
            xmlNodePtr sn = ttml_find_by_id(s->head, "style", tok, 0);
            if (sn)
                ttml_apply_element(s, sn, st, depth + 1);
        }
        av_free(dup);
    }
    ttml_apply_inline(n, st);
}

static void ttml_apply_region(TTMLContext *s, const char *id, TTMLStyle *st)
{
    xmlNodePtr rn = ttml_find_by_id(s->head, "region", id, 0);
    if (rn)
        ttml_apply_element(s, rn, st, 0);
}

static void ttml_ass_color(AVBPrint *bp, const char *tag, const char *atag,
                           int64_t c, int64_t old)
{
    if (c == old)
        return;
    if (c < 0) {
        av_bprintf(bp, "\\%s\\%s", tag, atag);
        return;
    }
    av_bprintf(bp, "\\%s&H%02X%02X%02X&", tag,
               (int)(c & 0xFF), (int)(c >> 8 & 0xFF), (int)(c >> 16 & 0xFF));
    av_bprintf(bp, "\\%s&H%02X&", atag, 255 - (int)(c >> 24 & 0xFF));
}

static void ttml_write_diff(AVBPrint *bp, const TTMLStyle *n, const TTMLStyle *o)
{
    AVBPrint tags;

    av_bprint_init(&tags, 0, AV_BPRINT_SIZE_UNLIMITED);
    if (n->bold != o->bold)
        av_bprintf(&tags, "\\b%d", FFMAX(n->bold, 0));
    if (n->italic != o->italic)
        av_bprintf(&tags, "\\i%d", FFMAX(n->italic, 0));
    if (n->underline != o->underline)
        av_bprintf(&tags, "\\u%d", FFMAX(n->underline, 0));
    ttml_ass_color(&tags, "1c", "1a", n->color, o->color);
    ttml_ass_color(&tags, "3c", "3a", n->bg, o->bg);
    if (strcmp(n->font, o->font))
        av_bprintf(&tags, "\\fn%s", n->font);
    if (tags.len)
        av_bprintf(bp, "{%s}", tags.str);
    av_bprint_finalize(&tags, NULL);
}

static void ttml_add_text(TTMLWalk *w, const char *t)
{
    for (; *t; t++) {
        if (*t == ' ' || *t == '\t' || *t == '\r' || *t == '\n') {
            if (!w->space)
                av_bprint_chars(w->bp, ' ', 1);
            w->space = 1;
            continue;
        }
        w->space = 0;
        if (*t == '{')
            av_bprintf(w->bp, "\\{{}");
        else if (*t == '\\')
            av_bprintf(w->bp, "\\\xe2\x81\xa0");
        else
            av_bprint_chars(w->bp, *t, 1);
    }
}

static void ttml_walk(TTMLWalk *w, xmlNodePtr parent, const TTMLStyle *ps,
                      int depth)
{
    xmlNodePtr n;

    if (depth > MAX_DEPTH)
        return;
    for (n = parent->children; n; n = n->next) {
        if (n->type == XML_TEXT_NODE || n->type == XML_CDATA_SECTION_NODE) {
            ttml_add_text(w, (const char *)n->content);
        } else if (ttml_is_elem(n, "br")) {
            av_bprintf(w->bp, "\\N");
            w->space = 1;
        } else if (n->type == XML_ELEMENT_NODE) {
            TTMLStyle st = *ps;
            xmlChar *reg = xmlGetNoNsProp(n, (const xmlChar *)"region");

            if (reg) {
                ttml_apply_region(w->s, (const char *)reg, &st);
                xmlFree(reg);
            }
            ttml_apply_element(w->s, n, &st, 0);
            if (!w->an && (st.text_align >= 0 || st.display_align >= 0)) {
                int h = st.text_align < 0 ? 2 : st.text_align;
                int v = st.display_align < 0 ? 2 : st.display_align;
                w->an = h + (v == 2 ? 0 : v == 1 ? 3 : 6);
            }
            ttml_write_diff(w->bp, &st, ps);
            ttml_walk(w, n, &st, depth + 1);
            ttml_write_diff(w->bp, ps, &st);
        }
    }
}

static int ttml_decode_frame(AVCodecContext *avctx, AVSubtitle *sub,
                             int *got_sub_ptr, const AVPacket *avpkt)
{
    TTMLContext *s = avctx->priv_data;
    AVBPrint doc_bp, out;
    TTMLWalk w = { .s = s, .bp = &out };
    xmlDocPtr doc;
    xmlNodePtr root;
    int ret = 0;

    if (!avpkt->size)
        return 0;

    av_bprint_init(&doc_bp, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprint_init(&out, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprintf(&doc_bp, "%s", s->prefix);
    av_bprint_append_data(&doc_bp, avpkt->data, avpkt->size);
    av_bprintf(&doc_bp, "</tt>");
    if (!av_bprint_is_complete(&doc_bp)) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    doc = xmlReadMemory(doc_bp.str, doc_bp.len, NULL, NULL,
                        XML_PARSE_NONET | XML_PARSE_NOERROR |
                        XML_PARSE_NOWARNING);
    if (!doc || !(root = xmlDocGetRootElement(doc))) {
        av_log(avctx, AV_LOG_ERROR, "Invalid TTML payload\n");
        xmlFreeDoc(doc);
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    ttml_walk(&w, root, &ttml_style_unset, 0);
    xmlFreeDoc(doc);

    if (!av_bprint_is_complete(&out)) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    while (out.len && out.str[out.len - 1] == ' ')
        out.str[--out.len] = 0;

    if (out.len) {
        AVBPrint line;
        const char *text = out.str + (out.str[0] == ' ');

        av_bprint_init(&line, 0, AV_BPRINT_SIZE_UNLIMITED);
        if (w.an)
            av_bprintf(&line, "{\\an%d}", w.an);
        av_bprintf(&line, "%s", text);
        if (!av_bprint_is_complete(&line))
            ret = AVERROR(ENOMEM);
        else
            ret = ff_ass_add_rect(sub, line.str, s->ass.readorder++, 0,
                                  NULL, NULL);
        av_bprint_finalize(&line, NULL);
        if (ret < 0)
            goto end;
    }
    *got_sub_ptr = sub->num_rects > 0;
    ret = avpkt->size;

end:
    av_bprint_finalize(&doc_bp, NULL);
    av_bprint_finalize(&out, NULL);
    return ret;
}

static int ttml_init(AVCodecContext *avctx)
{
    TTMLContext *s = avctx->priv_data;
    const char *params = TTML_DEFAULT_NAMESPACING;
    const char *head = "";
    AVBPrint bp;
    int ret;

    if (avctx->extradata_size > TTMLENC_EXTRADATA_SIGNATURE_SIZE &&
        !memcmp(avctx->extradata, TTMLENC_EXTRADATA_SIGNATURE,
                TTMLENC_EXTRADATA_SIGNATURE_SIZE)) {
        size_t left = avctx->extradata_size - TTMLENC_EXTRADATA_SIGNATURE_SIZE;
        const char *p = (const char *)avctx->extradata +
                        TTMLENC_EXTRADATA_SIGNATURE_SIZE;
        size_t len = av_strnlen(p, left);

        if (len < left) {
            params = p;
            left  -= len + 1;
            p     += len + 1;
            if (av_strnlen(p, left) < left)
                head = p;
        }
    }

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprintf(&bp, "<tt %s>", params);
    if (!av_bprint_is_complete(&bp))
        return AVERROR(ENOMEM);
    s->prefix = av_strdup(bp.str);
    if (!s->prefix) {
        av_bprint_finalize(&bp, NULL);
        return AVERROR(ENOMEM);
    }

    if (*head) {
        av_bprintf(&bp, "%s</tt>", head);
        if (av_bprint_is_complete(&bp))
            s->head_doc = xmlReadMemory(bp.str, bp.len, NULL, NULL,
                                        XML_PARSE_NONET | XML_PARSE_NOERROR |
                                        XML_PARSE_NOWARNING);
        if (s->head_doc)
            s->head = xmlDocGetRootElement(s->head_doc);
        else
            av_log(avctx, AV_LOG_WARNING,
                   "Could not parse the TTML head in the extradata\n");
    }
    av_bprint_finalize(&bp, NULL);

    ret = ff_ass_subtitle_header_default(avctx);
    return ret;
}

static av_cold int ttml_close(AVCodecContext *avctx)
{
    TTMLContext *s = avctx->priv_data;

    xmlFreeDoc(s->head_doc);
    av_freep(&s->prefix);
    return 0;
}

const FFCodec ff_ttml_decoder = {
    .p.name         = "ttml",
    CODEC_LONG_NAME("TTML subtitle"),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_TTML,
    FF_CODEC_DECODE_SUB_CB(ttml_decode_frame),
    .init           = ttml_init,
    .close          = ttml_close,
    .flush          = ff_ass_decoder_flush,
    .priv_data_size = sizeof(TTMLContext),
};

/*
 * TTML document parsing shared by the TTML demuxer and the MP4 demuxer
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

#include <libxml/parser.h>
#include <libxml/tree.h>

#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/dict.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "internal.h"
#include "ttml_parse.h"
#include "libavcodec/ttmlenc.h"

#define TTML_NS_TTS  "http://www.w3.org/ns/ttml#styling"
#define TTML_NS_TTP  "http://www.w3.org/ns/ttml#parameter"

typedef struct TTMLTimeParams {
    double frame_rate;      // effective frame rate, multiplier applied
    double tick_rate;
} TTMLTimeParams;

typedef struct TTMLWalkContext {
    void *log;
    FFTTMLParagraphCB cb;
    void *opaque;
} TTMLWalkContext;

static xmlChar *ttml_get_attr(xmlNodePtr node, const char *name, const char *ns)
{
    return ns ? xmlGetNsProp(node, (const xmlChar *)name, (const xmlChar *)ns)
              : xmlGetNoNsProp(node, (const xmlChar *)name);
}

static int ttml_is_elem(xmlNodePtr node, const char *name)
{
    /* elements without a namespace are accepted as TTML */
    return node->type == XML_ELEMENT_NODE &&
           (!node->ns || (node->ns->href &&
            !strcmp((const char *)node->ns->href, TTML_NS))) &&
           !strcmp((const char *)node->name, name);
}

static xmlNodePtr ttml_find_child(xmlNodePtr parent, const char *name)
{
    xmlNodePtr n;
    for (n = parent->children; n; n = n->next)
        if (ttml_is_elem(n, name))
            return n;
    return NULL;
}

static void ttml_parse_time_params(xmlNodePtr tt, TTMLTimeParams *tp)
{
    xmlChar *v;
    double frame_rate = 30.0, sub_frame_rate = 1.0, mult = 1.0;
    int have_rate = 0;

    if ((v = ttml_get_attr(tt, "frameRate", TTML_NS_TTP))) {
        double d = strtod((const char *)v, NULL);
        if (d > 0) {
            frame_rate = d;
            have_rate  = 1;
        }
        xmlFree(v);
    }
    if ((v = ttml_get_attr(tt, "subFrameRate", TTML_NS_TTP))) {
        double d = strtod((const char *)v, NULL);
        if (d > 0) {
            sub_frame_rate = d;
            have_rate      = 1;
        }
        xmlFree(v);
    }
    if ((v = ttml_get_attr(tt, "frameRateMultiplier", TTML_NS_TTP))) {
        int num, den;
        if (sscanf((const char *)v, "%d %d", &num, &den) == 2 &&
            num > 0 && den > 0)
            mult = (double)num / den;
        xmlFree(v);
    }

    tp->frame_rate = frame_rate * mult;
    tp->tick_rate  = have_rate ? frame_rate * sub_frame_rate : 1.0;

    if ((v = ttml_get_attr(tt, "tickRate", TTML_NS_TTP))) {
        double d = strtod((const char *)v, NULL);
        if (d > 0)
            tp->tick_rate = d;
        xmlFree(v);
    }
}

/**
 * Parse a TTML time expression (clock-time or offset-time) to milliseconds.
 */
static int ttml_parse_time(const char *s, const TTMLTimeParams *tp,
                           int64_t *out)
{
    int h, m, sec, n = 0;
    double ms;
    char *end;

    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        s++;

    if (sscanf(s, "%d:%d:%d%n", &h, &m, &sec, &n) >= 3 && n > 0) {
        s += n;
        ms = ((h * 60.0 + m) * 60.0 + sec) * 1000.0;
        if (*s == '.') {
            /* fraction of a second */
            char tmp[32];
            double frac;
            snprintf(tmp, sizeof(tmp), "0%s", s);
            frac = strtod(tmp, &end);
            s += end - tmp - 1;
            ms += frac * 1000.0;
        } else if (*s == ':') {
            /* frames, optionally with sub frames as a fraction */
            double frames = strtod(s + 1, &end);
            s = end;
            ms += frames * 1000.0 / tp->frame_rate;
        }
    } else {
        double v = strtod(s, &end);
        if (end == s)
            return AVERROR_INVALIDDATA;
        s = end;
        if (!strncmp(s, "ms", 2)) {
            ms = v;
            s += 2;
        } else if (*s == 'h') {
            ms = v * 3600000.0;
            s++;
        } else if (*s == 'm') {
            ms = v * 60000.0;
            s++;
        } else if (*s == 's') {
            ms = v * 1000.0;
            s++;
        } else if (*s == 'f') {
            ms = v * 1000.0 / tp->frame_rate;
            s++;
        } else if (*s == 't') {
            ms = v * 1000.0 / tp->tick_rate;
            s++;
        } else {
            return AVERROR_INVALIDDATA;
        }
    }

    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        s++;
    if (*s)
        return AVERROR_INVALIDDATA;

    *out = llrint(ms);
    return 0;
}

static int ttml_get_time_attr(void *log, xmlNodePtr node,
                              const char *name, const TTMLTimeParams *tp,
                              int64_t *out)
{
    xmlChar *v = ttml_get_attr(node, name, NULL);
    int ret;

    if (!v)
        return 1;
    ret = ttml_parse_time((const char *)v, tp, out);
    if (ret < 0)
        av_log(log, AV_LOG_WARNING, "Invalid TTML %s value '%s'\n", name, v);
    xmlFree(v);
    return ret < 0;
}

/**
 * Resolve the interval of a timed element. Times are relative to the begin of
 * the parent (parallel time container); missing values are inherited from
 * the parent. An end of AV_NOPTS_VALUE means unknown.
 */
static void ttml_resolve_interval(void *log, xmlNodePtr node,
                                  const TTMLTimeParams *tp,
                                  int64_t pbegin, int64_t pend,
                                  int64_t *begin, int64_t *end)
{
    int64_t rel, dur = AV_NOPTS_VALUE;

    *begin = pbegin;
    *end   = pend;

    if (!ttml_get_time_attr(log, node, "begin", tp, &rel))
        *begin = pbegin + rel;
    if (!ttml_get_time_attr(log, node, "end", tp, &rel))
        *end = pbegin + rel;
    if (!ttml_get_time_attr(log, node, "dur", tp, &dur)) {
        if (*end == AV_NOPTS_VALUE || *begin + dur < *end)
            *end = *begin + dur;
    }
    if (pend != AV_NOPTS_VALUE && *end != AV_NOPTS_VALUE && *end > pend)
        *end = pend;
}

/**
 * Space separated list of inherited style ids plus the own ones.
 */
static void ttml_append_style(AVBPrint *bp, const xmlChar *v)
{
    if (!v || !*v)
        return;
    if (bp->len)
        av_bprint_chars(bp, ' ', 1);
    av_bprintf(bp, "%s", v);
}

static void ttml_print_attr(AVBPrint *bp, const char *name, const char *value)
{
    av_bprintf(bp, " %s=\"", name);
    av_bprint_escape(bp, value, NULL, AV_ESCAPE_MODE_XML,
                     AV_ESCAPE_FLAG_XML_DOUBLE_QUOTES);
    av_bprint_chars(bp, '"', 1);
}

static int ttml_add_paragraph(TTMLWalkContext *wc,
                              xmlDocPtr doc, xmlNodePtr p,
                              int64_t begin, int64_t end,
                              const char *style, const char *region)
{
    AVBPrint bp, own_style;
    xmlAttrPtr attr;
    xmlNodePtr c;
    xmlBufferPtr xb;
    xmlChar *v;
    int wrap, ret = 0;

    if (begin == AV_NOPTS_VALUE)
        return 0;

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprint_init(&own_style, 0, AV_BPRINT_SIZE_UNLIMITED);

    /* style: inherited ones first, then the paragraph's own */
    ttml_append_style(&own_style, (const xmlChar *)style);
    if ((v = ttml_get_attr(p, "style", NULL))) {
        ttml_append_style(&own_style, v);
        xmlFree(v);
    }
    if ((v = ttml_get_attr(p, "region", NULL))) {
        region = av_strdup((const char *)v);
        xmlFree(v);
        if (!region) {
            ret = AVERROR(ENOMEM);
            goto end;
        }
    } else if (region) {
        region = av_strdup(region);
        if (!region) {
            ret = AVERROR(ENOMEM);
            goto end;
        }
    }

    /* inline styling attributes on the paragraph itself */
    wrap = own_style.len || region;
    for (attr = p->properties; attr; attr = attr->next)
        if (attr->ns && attr->ns->href &&
            !strcmp((const char *)attr->ns->href, TTML_NS_TTS))
            wrap = 1;

    if (wrap) {
        av_bprintf(&bp, "<span");
        if (own_style.len)
            ttml_print_attr(&bp, "style", own_style.str);
        if (region)
            ttml_print_attr(&bp, "region", region);
        for (attr = p->properties; attr; attr = attr->next) {
            if (attr->ns && attr->ns->href &&
                !strcmp((const char *)attr->ns->href, TTML_NS_TTS)) {
                char name[128];
                v = xmlNodeGetContent((xmlNodePtr)attr);
                snprintf(name, sizeof(name), "%s:%s",
                         attr->ns->prefix ? (const char *)attr->ns->prefix : "tts",
                         attr->name);
                ttml_print_attr(&bp, name, v ? (const char *)v : "");
                xmlFree(v);
            }
        }
        av_bprintf(&bp, ">");
    }

    xb = xmlBufferCreate();
    if (!xb) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    for (c = p->children; c; c = c->next)
        xmlNodeDump(xb, doc, c, 0, 0);
    av_bprintf(&bp, "%s", xb->content ? (const char *)xb->content : "");
    xmlBufferFree(xb);

    if (wrap)
        av_bprintf(&bp, "</span>");

    if (!av_bprint_is_complete(&bp)) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = wc->cb(wc->opaque, bp.str, bp.len, begin, end, xmlGetLineNo(p));

end:
    av_free((void *)region);
    av_bprint_finalize(&bp, NULL);
    av_bprint_finalize(&own_style, NULL);
    return ret;
}

static int ttml_walk(TTMLWalkContext *wc, xmlDocPtr doc,
                     xmlNodePtr parent, const TTMLTimeParams *tp,
                     int64_t pbegin, int64_t pend,
                     const char *style, const char *region, int depth)
{
    xmlNodePtr n;
    int ret;

    if (depth > 64)
        return AVERROR_INVALIDDATA;

    for (n = parent->children; n; n = n->next) {
        int64_t begin, end;
        int is_p = ttml_is_elem(n, "p");

        if (!is_p && !ttml_is_elem(n, "div") && !ttml_is_elem(n, "body"))
            continue;

        ttml_resolve_interval(wc->log, n, tp, pbegin, pend, &begin, &end);

        if (is_p) {
            ret = ttml_add_paragraph(wc, doc, n, begin, end, style, region);
        } else {
            AVBPrint st;
            xmlChar *v;
            char *reg = NULL;

            av_bprint_init(&st, 0, AV_BPRINT_SIZE_UNLIMITED);
            ttml_append_style(&st, (const xmlChar *)style);
            if ((v = ttml_get_attr(n, "style", NULL))) {
                ttml_append_style(&st, v);
                xmlFree(v);
            }
            if ((v = ttml_get_attr(n, "region", NULL))) {
                reg = av_strdup((const char *)v);
                xmlFree(v);
            } else if (region) {
                reg = av_strdup(region);
            }
            if (!av_bprint_is_complete(&st)) {
                ret = AVERROR(ENOMEM);
            } else {
                ret = ttml_walk(wc, doc, n, tp, begin, end,
                                st.str, reg, depth + 1);
            }
            av_free(reg);
            av_bprint_finalize(&st, NULL);
        }
        if (ret < 0)
            return ret;
    }
    return 0;
}

static int ttml_set_extradata(AVCodecParameters *par, AVDictionary **metadata,
                              xmlDocPtr doc,
                              xmlNodePtr tt)
{
    AVBPrint tt_params, head_bp;
    xmlNsPtr ns;
    xmlAttrPtr attr;
    xmlNodePtr head;
    xmlBufferPtr xb;
    size_t sig = TTMLENC_EXTRADATA_SIGNATURE_SIZE;
    uint8_t *ed;
    int ret = 0;

    av_bprint_init(&tt_params, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprint_init(&head_bp,   0, AV_BPRINT_SIZE_UNLIMITED);

    /* the muxers need the TTML namespace, a document without one gets it */
    if (!tt->ns)
        av_bprintf(&tt_params, "  xmlns=\"" TTML_NS "\"\n");
    for (ns = tt->nsDef; ns; ns = ns->next) {
        av_bprintf(&tt_params, "  xmlns%s%s=\"",
                   ns->prefix ? ":" : "", ns->prefix ? (const char *)ns->prefix : "");
        av_bprint_escape(&tt_params, (const char *)ns->href, NULL,
                         AV_ESCAPE_MODE_XML, AV_ESCAPE_FLAG_XML_DOUBLE_QUOTES);
        av_bprintf(&tt_params, "\"\n");
    }
    for (attr = tt->properties; attr; attr = attr->next) {
        xmlChar *v;
        if (attr->ns && attr->ns->prefix &&
            !strcmp((const char *)attr->ns->prefix, "xml") &&
            !strcmp((const char *)attr->name, "lang")) {
            v = xmlNodeGetContent((xmlNodePtr)attr);
            if (v && *v && metadata)
                av_dict_set(metadata, "language", (const char *)v, 0);
            xmlFree(v);
            continue;
        }
        v = xmlNodeGetContent((xmlNodePtr)attr);
        av_bprintf(&tt_params, "  %s%s%s=\"",
                   attr->ns && attr->ns->prefix ? (const char *)attr->ns->prefix : "",
                   attr->ns && attr->ns->prefix ? ":" : "",
                   attr->name);
        av_bprint_escape(&tt_params, v ? (const char *)v : "", NULL,
                         AV_ESCAPE_MODE_XML, AV_ESCAPE_FLAG_XML_DOUBLE_QUOTES);
        av_bprintf(&tt_params, "\"\n");
        xmlFree(v);
    }

    head = ttml_find_child(tt, "head");
    if (head) {
        xb = xmlBufferCreate();
        if (!xb) {
            ret = AVERROR(ENOMEM);
            goto end;
        }
        xmlNodeDump(xb, doc, head, 1, 1);
        av_bprintf(&head_bp, "%s\n", xb->content ? (const char *)xb->content : "");
        xmlBufferFree(xb);
    }

    if (!av_bprint_is_complete(&tt_params) || !av_bprint_is_complete(&head_bp)) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = ff_alloc_extradata(par,
                             sig + tt_params.len + 1 + head_bp.len + 1);
    if (ret < 0)
        goto end;
    ed = par->extradata;
    memcpy(ed, TTMLENC_EXTRADATA_SIGNATURE, sig);
    ed += sig;
    memcpy(ed, tt_params.str, tt_params.len + 1);   // includes the NUL
    ed += tt_params.len + 1;
    memcpy(ed, head_bp.str, head_bp.len + 1);

end:
    av_bprint_finalize(&tt_params, NULL);
    av_bprint_finalize(&head_bp, NULL);
    return ret;
}
int ff_ttml_parse(void *log_ctx, const uint8_t *buf, int size,
                  AVCodecParameters *par, AVDictionary **metadata,
                  FFTTMLParagraphCB cb, void *opaque)
{
    TTMLWalkContext wc = { .log = log_ctx, .cb = cb, .opaque = opaque };
    xmlDocPtr doc;
    xmlNodePtr root, body;
    TTMLTimeParams tp;
    int ret = 0;

    doc = xmlReadMemory((const char *)buf, size, NULL, NULL,
                        XML_PARSE_NONET | XML_PARSE_NOERROR | XML_PARSE_NOWARNING);
    if (!doc) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to parse TTML document\n");
        return AVERROR_INVALIDDATA;
    }
    root = xmlDocGetRootElement(doc);
    if (!root || !ttml_is_elem(root, "tt")) {
        av_log(log_ctx, AV_LOG_ERROR, "Root element is not a TTML <tt>\n");
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    if (par) {
        ret = ttml_set_extradata(par, metadata, doc, root);
        if (ret < 0)
            goto end;
    }
    if (!cb)
        goto end;

    ttml_parse_time_params(root, &tp);

    body = ttml_find_child(root, "body");
    if (body) {
        int64_t begin, end;
        AVBPrint st_bp;
        xmlChar *v;
        char *region = NULL;

        /* the body is a child of the root, which starts at 0 */
        ttml_resolve_interval(log_ctx, body, &tp, 0, AV_NOPTS_VALUE, &begin, &end);

        av_bprint_init(&st_bp, 0, AV_BPRINT_SIZE_UNLIMITED);
        if ((v = ttml_get_attr(body, "style", NULL))) {
            ttml_append_style(&st_bp, v);
            xmlFree(v);
        }
        if ((v = ttml_get_attr(body, "region", NULL))) {
            region = av_strdup((const char *)v);
            xmlFree(v);
        }
        /* walk the children of the body directly, as its own interval
         * is already resolved */
        ret = ttml_walk(&wc, doc, body, &tp, begin, end,
                        st_bp.str, region, 0);
        av_free(region);
        av_bprint_finalize(&st_bp, NULL);
    }

end:
    xmlFreeDoc(doc);
    return ret;
}

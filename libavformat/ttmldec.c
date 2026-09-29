/*
 * TTML subtitle demuxer
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
 * TTML subtitle demuxer
 *
 * Emits one packet per <p> element. The packet payload is the inner XML of
 * the paragraph, wrapped in a <span> carrying the style and region of the
 * paragraph if there are any. The <tt> attributes and <head> element are
 * stored in the extradata in the format understood by the ttml muxer.
 *
 * @see https://www.w3.org/TR/ttml1/
 * @see https://www.w3.org/TR/ttml2/
 * @see https://www.w3.org/TR/ttml-imsc/rec
 */

#include <libxml/parser.h>
#include <libxml/tree.h>

#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/rational.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "subtitles.h"
#include "libavcodec/ttmlenc.h"

#define TTML_NS      "http://www.w3.org/ns/ttml"
#define TTML_NS_TTS  "http://www.w3.org/ns/ttml#styling"
#define TTML_NS_TTP  "http://www.w3.org/ns/ttml#parameter"

#define TTML_MAX_SIZE (64 * 1024 * 1024)

typedef struct TTMLTimeParams {
    double frame_rate;      // effective frame rate, multiplier applied
    double tick_rate;
} TTMLTimeParams;

typedef struct TTMLContext {
    FFDemuxSubtitlesQueue q;
} TTMLContext;

static int ttml_probe(const AVProbeData *p)
{
    const char *buf = p->buf, *end = p->buf + p->buf_size;
    const char *tt = NULL, *q;

    if (p->buf_size >= 3 && !memcmp(buf, "\xEF\xBB\xBF", 3))
        buf += 3;

    for (q = buf; q < end - 4; q++) {
        if (*q != '<')
            continue;
        if (q[1] == '?' || q[1] == '!')
            continue;
        /* the root element, possibly with a prefix */
        {
            const char *n = q + 1;
            const char *colon = memchr(n, ':', FFMIN(end - n, 32));
            const char *sp = n + strcspn(n, " \t\r\n>/");
            if (colon && colon < sp)
                n = colon + 1;
            if (end - n >= 3 && !strncmp(n, "tt", 2) &&
                strchr(" \t\r\n>", n[2]))
                tt = q;
        }
        break;
    }
    if (!tt)
        return 0;

    for (q = tt; q < end - (int)sizeof(TTML_NS); q++)
        if (!strncmp(q, TTML_NS, sizeof(TTML_NS) - 1))
            return AVPROBE_SCORE_MAX - 1;

    /* a <tt> root with no namespace at all: only trust it with the extension */
    return AVPROBE_SCORE_EXTENSION + 1;
}

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

static int ttml_get_time_attr(AVFormatContext *s, xmlNodePtr node,
                              const char *name, const TTMLTimeParams *tp,
                              int64_t *out)
{
    xmlChar *v = ttml_get_attr(node, name, NULL);
    int ret;

    if (!v)
        return 1;
    ret = ttml_parse_time((const char *)v, tp, out);
    if (ret < 0)
        av_log(s, AV_LOG_WARNING, "Invalid TTML %s value '%s'\n", name, v);
    xmlFree(v);
    return ret < 0;
}

/**
 * Resolve the interval of a timed element. Times are relative to the begin of
 * the parent (parallel time container); missing values are inherited from
 * the parent. An end of AV_NOPTS_VALUE means unknown.
 */
static void ttml_resolve_interval(AVFormatContext *s, xmlNodePtr node,
                                  const TTMLTimeParams *tp,
                                  int64_t pbegin, int64_t pend,
                                  int64_t *begin, int64_t *end)
{
    int64_t rel, dur = AV_NOPTS_VALUE;

    *begin = pbegin;
    *end   = pend;

    if (!ttml_get_time_attr(s, node, "begin", tp, &rel))
        *begin = pbegin + rel;
    if (!ttml_get_time_attr(s, node, "end", tp, &rel))
        *end = pbegin + rel;
    if (!ttml_get_time_attr(s, node, "dur", tp, &dur)) {
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

static int ttml_add_paragraph(AVFormatContext *s, TTMLContext *ttml,
                              xmlDocPtr doc, xmlNodePtr p,
                              int64_t begin, int64_t end,
                              const char *style, const char *region)
{
    AVBPrint bp, own_style;
    xmlAttrPtr attr;
    xmlNodePtr c;
    xmlBufferPtr xb;
    xmlChar *v;
    AVPacket *sub;
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

    sub = ff_subtitles_queue_insert_bprint(&ttml->q, &bp, 0);
    if (!sub) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    sub->pos      = xmlGetLineNo(p);
    sub->pts      = begin;
    sub->duration = end == AV_NOPTS_VALUE ? -1 : FFMAX(end - begin, 0);

end:
    av_free((void *)region);
    av_bprint_finalize(&bp, NULL);
    av_bprint_finalize(&own_style, NULL);
    return ret;
}

static int ttml_walk(AVFormatContext *s, TTMLContext *ttml, xmlDocPtr doc,
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

        ttml_resolve_interval(s, n, tp, pbegin, pend, &begin, &end);

        if (is_p) {
            ret = ttml_add_paragraph(s, ttml, doc, n, begin, end, style, region);
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
                ret = ttml_walk(s, ttml, doc, n, tp, begin, end,
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

static int ttml_set_extradata(AVFormatContext *s, AVStream *st, xmlDocPtr doc,
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
            if (v && *v)
                av_dict_set(&st->metadata, "language", (const char *)v, 0);
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

    ret = ff_alloc_extradata(st->codecpar,
                             sig + tt_params.len + 1 + head_bp.len + 1);
    if (ret < 0)
        goto end;
    ed = st->codecpar->extradata;
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

static int ttml_read_header(AVFormatContext *s)
{
    TTMLContext *ttml = s->priv_data;
    AVBPrint doc_buf;
    AVStream *st;
    xmlDocPtr doc = NULL;
    xmlNodePtr root, body;
    TTMLTimeParams tp;
    uint8_t chunk[8192];
    int ret;

    av_bprint_init(&doc_buf, 0, AV_BPRINT_SIZE_UNLIMITED);
    for (;;) {
        ret = avio_read(s->pb, chunk, sizeof(chunk));
        if (ret == AVERROR_EOF || ret == 0)
            break;
        if (ret < 0)
            goto end;
        av_bprint_append_data(&doc_buf, (const char *)chunk, ret);
        if (doc_buf.len > TTML_MAX_SIZE) {
            ret = AVERROR_INVALIDDATA;
            goto end;
        }
    }
    if (!av_bprint_is_complete(&doc_buf)) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    doc = xmlReadMemory(doc_buf.str, doc_buf.len, NULL, NULL,
                        XML_PARSE_NONET | XML_PARSE_NOERROR | XML_PARSE_NOWARNING);
    if (!doc) {
        av_log(s, AV_LOG_ERROR, "Failed to parse TTML document\n");
        ret = AVERROR_INVALIDDATA;
        goto end;
    }
    root = xmlDocGetRootElement(doc);
    if (!root || !ttml_is_elem(root, "tt")) {
        av_log(s, AV_LOG_ERROR, "Root element is not a TTML <tt>\n");
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    avpriv_set_pts_info(st, 64, 1, 1000);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id   = AV_CODEC_ID_TTML;

    ret = ttml_set_extradata(s, st, doc, root);
    if (ret < 0)
        goto end;

    ttml_parse_time_params(root, &tp);

    body = ttml_find_child(root, "body");
    if (body) {
        int64_t begin, end;
        /* the body is a child of the root, which starts at 0 */
        ttml_resolve_interval(s, body, &tp, 0, AV_NOPTS_VALUE, &begin, &end);
        {
            AVBPrint st_bp;
            xmlChar *v;
            char *region = NULL;

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
            ret = ttml_walk(s, ttml, doc, body, &tp, begin, end,
                            st_bp.str, region, 0);
            av_free(region);
            av_bprint_finalize(&st_bp, NULL);
        }
        if (ret < 0)
            goto end;
    }

    ff_subtitles_queue_finalize(s, &ttml->q);

end:
    if (doc)
        xmlFreeDoc(doc);
    av_bprint_finalize(&doc_buf, NULL);
    if (ret < 0)
        ff_subtitles_queue_clean(&ttml->q);
    return ret < 0 ? ret : 0;
}

static int ttml_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    TTMLContext *ttml = s->priv_data;
    return ff_subtitles_queue_read_packet(&ttml->q, pkt);
}

static int ttml_read_seek(AVFormatContext *s, int stream_index,
                          int64_t min_ts, int64_t ts, int64_t max_ts, int flags)
{
    TTMLContext *ttml = s->priv_data;
    return ff_subtitles_queue_seek(&ttml->q, s, stream_index,
                                   min_ts, ts, max_ts, flags);
}

static int ttml_read_close(AVFormatContext *s)
{
    TTMLContext *ttml = s->priv_data;
    ff_subtitles_queue_clean(&ttml->q);
    return 0;
}

const FFInputFormat ff_ttml_demuxer = {
    .p.name         = "ttml",
    .p.long_name    = NULL_IF_CONFIG_SMALL("TTML subtitle"),
    .p.mime_type    = "application/ttml+xml",
    .p.extensions   = "ttml,dfxp",
    .priv_data_size = sizeof(TTMLContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = ttml_probe,
    .read_header    = ttml_read_header,
    .read_packet    = ttml_read_packet,
    .read_seek2     = ttml_read_seek,
    .read_close     = ttml_read_close,
};

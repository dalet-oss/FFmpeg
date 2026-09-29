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

#include "libavutil/bprint.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "subtitles.h"
#include "ttml_parse.h"

#define TTML_MAX_SIZE (64 * 1024 * 1024)

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


static int ttml_add_paragraph(void *opaque, const char *text, int len,
                              int64_t begin, int64_t end, int64_t line)
{
    TTMLContext *ttml = opaque;
    AVPacket *sub = ff_subtitles_queue_insert(&ttml->q, text, len, 0);

    if (!sub)
        return AVERROR(ENOMEM);
    sub->pos      = line;
    sub->pts      = begin;
    sub->duration = end == AV_NOPTS_VALUE ? -1 : FFMAX(end - begin, 0);
    return 0;
}

static int ttml_read_header(AVFormatContext *s)
{
    TTMLContext *ttml = s->priv_data;
    AVBPrint doc_buf;
    AVStream *st;
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

    st = avformat_new_stream(s, NULL);
    if (!st) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    avpriv_set_pts_info(st, 64, 1, 1000);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id   = AV_CODEC_ID_TTML;

    ret = ff_ttml_parse(s, (const uint8_t *)doc_buf.str, doc_buf.len,
                        st->codecpar, &st->metadata, ttml_add_paragraph, ttml);
    if (ret < 0)
        goto end;

    ff_subtitles_queue_finalize(s, &ttml->q);

end:
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

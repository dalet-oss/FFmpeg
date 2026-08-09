/*
 * MP4 muxer WebVTT (ISO/IEC 14496-30) helpers
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

#include <stdint.h>
#include <string.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "isom.h"
#include "movenc.h"
#include "movenc_wvtt.h"
#include "libavcodec/packet_internal.h"

/* The WebVTT file header the cues of the track originate from. Everything
 * before the first cue of a WebVTT file belongs to it, but as the WebVTT
 * demuxer does not keep the header around, the mandatory signature is all we
 * are able to write. */
static const char webvtt_file_header[] = "WEBVTT";

typedef struct WVTTCue {
    AVPacket *pkt;      /* owner of the payload and side data below */
    int64_t start;      /* cue start, in track time base, clipped to the fragment */
    int64_t end;        /* cue end, in track time base, clipped to the fragment */
    int64_t orig_start; /* cue start before clipping, written as 'ctim' */
    const uint8_t *id;
    size_t id_size;
    const uint8_t *settings;
    size_t settings_size;
    const uint8_t *payload;
    size_t payload_size;
} WVTTCue;

void ff_mov_write_wvtt_config(AVIOContext *pb, MOVTrack *track)
{
    avio_wb32(pb, 8 + sizeof(webvtt_file_header) - 1);
    avio_wl32(pb, MKTAG('v','t','t','C'));
    avio_write(pb, (const uint8_t *)webvtt_file_header,
               sizeof(webvtt_file_header) - 1);
}

static void wvtt_write_box(AVIOContext *pb, uint32_t tag,
                           const uint8_t *data, size_t size)
{
    avio_wb32(pb, 8 + size);
    avio_wl32(pb, tag); // stored byteswapped, as everywhere else in the muxer
    if (size)
        avio_write(pb, data, size);
}

/* Format a time as the WebVTT timestamp 'ctim' carries, hh:mm:ss.ttt. */
static void wvtt_format_time(char *buf, size_t buf_size, MOVTrack *track,
                             int64_t ts)
{
    int64_t ms = av_rescale_q(ts, track->st->time_base, (AVRational){ 1, 1000 });
    int64_t s  = ms / 1000;

    snprintf(buf, buf_size, "%02"PRId64":%02"PRId64":%02"PRId64".%03"PRId64,
             s / 3600, (s / 60) % 60, s % 60, ms % 1000);
}

static void wvtt_write_cue(AVIOContext *pb, MOVTrack *track,
                           const WVTTCue *cue, int64_t sample_start)
{
    char ctim[32] = { 0 };
    size_t ctim_size = 0;
    uint32_t size = 8;

    /* The cue started before this sample, either because another cue starting
     * or ending in the middle of it forced a new sample, or because it reaches
     * over the start of this fragment. 14496-30 has 'ctim' preserve its
     * original timing for that case. */
    if (cue->orig_start < sample_start) {
        wvtt_format_time(ctim, sizeof(ctim), track, cue->orig_start);
        ctim_size = strlen(ctim);
    }

    if (cue->id_size)
        size += 8 + cue->id_size;
    if (ctim_size)
        size += 8 + ctim_size;
    if (cue->settings_size)
        size += 8 + cue->settings_size;
    size += 8 + cue->payload_size;

    avio_wb32(pb, size);
    avio_wl32(pb, MKTAG('v','t','t','c'));

    /* Box order as defined by the VTTCueBox syntax. */
    if (cue->id_size)
        wvtt_write_box(pb, MKTAG('i','d','e','n'), cue->id, cue->id_size);
    if (ctim_size)
        wvtt_write_box(pb, MKTAG('c','t','i','m'), (const uint8_t *)ctim, ctim_size);
    if (cue->settings_size)
        wvtt_write_box(pb, MKTAG('s','t','t','g'), cue->settings, cue->settings_size);
    wvtt_write_box(pb, MKTAG('p','a','y','l'), cue->payload, cue->payload_size);
}

/* Resolve the time window the samples of this fragment have to cover. */
static void wvtt_get_window(AVFormatContext *s, MOVTrack *track,
                            int64_t *out_start, int64_t *out_end)
{
    MOVMuxContext *mov = s->priv_data;
    int64_t start = AV_NOPTS_VALUE, end = AV_NOPTS_VALUE;

    if (mov->flags & FF_MOV_FLAG_FRAGMENT)
        ff_mov_calculate_fragment_window(s, track, &start, &end);

    if (end == AV_NOPTS_VALUE) {
        /* Nothing declared the end of this fragment, so cover exactly the cues
         * we have. Keeps non fragmented output and the first fragment working
         * without the caller having to know anything. */
        for (const PacketListEntry *e = track->squashed_packet_queue.head; e; e = e->next)
            end = end == AV_NOPTS_VALUE ? e->pkt.pts + e->pkt.duration
                                        : FFMAX(end, e->pkt.pts + e->pkt.duration);
    }

    if (start == AV_NOPTS_VALUE) {
        const PacketListEntry *first = track->squashed_packet_queue.head;

        /* Pick up where the previous fragment left off, so that the samples of
         * the track stay contiguous, as 14496-30 requires. */
        if (track->end_pts != AV_NOPTS_VALUE)
            start = track->end_pts;
        else if (first)
            start = first->pkt.pts;
        else
            start = 0;
    }

    *out_start = start;
    *out_end   = end;
}

static int wvtt_add_time(int64_t **times, int *nb_times, int *times_size,
                         int64_t time)
{
    for (int i = 0; i < *nb_times; i++)
        if ((*times)[i] == time)
            return 0;

    if (*nb_times >= *times_size) {
        int new_size = *times_size ? *times_size * 2 : 16;
        int64_t *tmp = av_realloc_array(*times, new_size, sizeof(**times));
        if (!tmp)
            return AVERROR(ENOMEM);
        *times = tmp;
        *times_size = new_size;
    }

    (*times)[(*nb_times)++] = time;
    return 0;
}

static int wvtt_cmp_time(const void *a, const void *b)
{
    int64_t diff = *(const int64_t *)a - *(const int64_t *)b;
    return diff < 0 ? -1 : diff > 0 ? 1 : 0;
}

/* Take every cue overlapping the fragment out of the queue, cutting the ones
 * reaching over its end and putting their remainder back. */
static int wvtt_take_cues(AVFormatContext *s, MOVTrack *track,
                          int64_t window_start, int64_t window_end,
                          WVTTCue **out_cues, int *out_nb_cues)
{
    PacketList back_to_queue_list = { 0 };
    WVTTCue *cues = NULL;
    int nb_cues = 0, cues_size = 0;
    AVPacket *pkt = NULL;
    int ret = 0;

    while (track->squashed_packet_queue.head) {
        const AVPacket *head = &track->squashed_packet_queue.head->pkt;
        int64_t orig_start, cue_end;
        size_t side_size;

        if (head->pts >= window_end)
            break; // starts after this fragment, leave it for the next one

        if (!pkt && !(pkt = av_packet_alloc())) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        ret = avpriv_packet_list_get(&track->squashed_packet_queue, pkt);
        if (ret < 0)
            goto fail;

        /* A cue put back by a previous fragment carries the start it had
         * before being cut in its dts, so that 'ctim' can still be written. */
        orig_start = (pkt->dts != AV_NOPTS_VALUE && pkt->dts < pkt->pts) ?
                     pkt->dts : pkt->pts;
        cue_end    = pkt->pts + pkt->duration;

        if (cue_end <= window_start) {
            av_log(s, AV_LOG_WARNING,
                   "Very late WebVTT cue in queue, dropping cue with "
                   "pts: %"PRId64", duration: %"PRId64"\n",
                   pkt->pts, pkt->duration);
            av_packet_unref(pkt);
            continue;
        }

        if (cue_end > window_end) {
            /* Reaches over this fragment: cut it here and queue the rest. */
            ret = avpriv_packet_list_put(&back_to_queue_list, pkt, av_packet_ref,
                                         FF_PACKETLIST_FLAG_PREPEND);
            if (ret < 0)
                goto fail;

            back_to_queue_list.head->pkt.pts      = window_end;
            back_to_queue_list.head->pkt.dts      = orig_start;
            back_to_queue_list.head->pkt.duration = cue_end - window_end;
        }

        if (nb_cues >= cues_size) {
            int new_size = cues_size ? cues_size * 2 : 8;
            WVTTCue *tmp = av_realloc_array(cues, new_size, sizeof(*cues));
            if (!tmp) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            cues = tmp;
            cues_size = new_size;
        }

        cues[nb_cues] = (WVTTCue){
            .pkt        = pkt,
            .start      = FFMAX(pkt->pts, window_start),
            .end        = FFMIN(cue_end,  window_end),
            .orig_start = orig_start,
            .payload    = pkt->data,
            .payload_size = pkt->size,
        };

        cues[nb_cues].id = av_packet_get_side_data(pkt, AV_PKT_DATA_WEBVTT_IDENTIFIER,
                                                   &side_size);
        cues[nb_cues].id_size = cues[nb_cues].id ? side_size : 0;
        cues[nb_cues].settings = av_packet_get_side_data(pkt, AV_PKT_DATA_WEBVTT_SETTINGS,
                                                         &side_size);
        cues[nb_cues].settings_size = cues[nb_cues].settings ? side_size : 0;

        /* The demuxers hand us the cue text with the newline separating it from
         * the next cue still attached. */
        while (cues[nb_cues].payload_size &&
               (cues[nb_cues].payload[cues[nb_cues].payload_size - 1] == '\n' ||
                cues[nb_cues].payload[cues[nb_cues].payload_size - 1] == '\r'))
            cues[nb_cues].payload_size--;

        nb_cues++;
        pkt = NULL; // owned by the cue now
    }

fail:
    av_packet_free(&pkt);

    /* Whether we succeeded or not, the cues we cut have to go back. */
    if (back_to_queue_list.head) {
        AVPacket *back = av_packet_alloc();

        if (!back) {
            avpriv_packet_list_free(&back_to_queue_list);
            ret = AVERROR(ENOMEM);
        } else {
            while (!avpriv_packet_list_get(&back_to_queue_list, back)) {
                int put_ret = avpriv_packet_list_put(&track->squashed_packet_queue,
                                                     back, av_packet_ref,
                                                     FF_PACKETLIST_FLAG_PREPEND);
                av_packet_unref(back);
                if (put_ret < 0) {
                    avpriv_packet_list_free(&back_to_queue_list);
                    ret = put_ret;
                    break;
                }
            }
            av_packet_free(&back);
        }
    }

    if (ret < 0) {
        for (int i = 0; i < nb_cues; i++)
            av_packet_free(&cues[i].pkt);
        av_freep(&cues);
        nb_cues = 0;
    }

    *out_cues    = cues;
    *out_nb_cues = nb_cues;
    return ret;
}

int ff_mov_generate_wvtt_samples(AVFormatContext *s, MOVTrack *track,
                                 PacketList *out)
{
    WVTTCue *cues = NULL;
    int64_t *times = NULL;
    int nb_cues = 0, nb_times = 0, times_size = 0;
    int64_t window_start, window_end;
    AVPacket *sample = NULL;
    int ret;

    wvtt_get_window(s, track, &window_start, &window_end);

    if (window_end == AV_NOPTS_VALUE || window_end <= window_start)
        return 0; // nothing to cover

    ret = wvtt_take_cues(s, track, window_start, window_end, &cues, &nb_cues);
    if (ret < 0)
        goto end;

    /* Every cue boundary starts a new sample, so that no sample ever has a cue
     * appearing or disappearing in the middle of it. */
    if ((ret = wvtt_add_time(&times, &nb_times, &times_size, window_start)) < 0 ||
        (ret = wvtt_add_time(&times, &nb_times, &times_size, window_end)) < 0)
        goto end;

    for (int i = 0; i < nb_cues; i++)
        if ((ret = wvtt_add_time(&times, &nb_times, &times_size, cues[i].start)) < 0 ||
            (ret = wvtt_add_time(&times, &nb_times, &times_size, cues[i].end)) < 0)
            goto end;

    qsort(times, nb_times, sizeof(*times), wvtt_cmp_time);

    for (int i = 0; i + 1 < nb_times; i++) {
        int64_t sample_start = times[i], sample_end = times[i + 1];
        AVIOContext *pb = NULL;
        uint8_t *buf = NULL;
        int size, nb_active = 0;

        if ((ret = avio_open_dyn_buf(&pb)) < 0)
            goto end;

        for (int j = 0; j < nb_cues; j++) {
            if (cues[j].start > sample_start || cues[j].end < sample_end)
                continue;
            wvtt_write_cue(pb, track, &cues[j], sample_start);
            nb_active++;
        }

        /* 14496-30 covers the stretches of time with nothing to display with
         * an empty cue rather than leaving a gap in the samples. */
        if (!nb_active)
            wvtt_write_box(pb, MKTAG('v','t','t','e'), NULL, 0);

        size = avio_close_dyn_buf(pb, &buf);
        if (size < 0) {
            av_freep(&buf);
            ret = size;
            goto end;
        }

        if (!(sample = av_packet_alloc())) {
            av_freep(&buf);
            ret = AVERROR(ENOMEM);
            goto end;
        }

        if ((ret = av_packet_from_data(sample, buf, size)) < 0) {
            av_freep(&buf);
            goto end;
        }

        sample->pts = sample->dts = sample_start;
        sample->duration = sample_end - sample_start;
        sample->flags |= AV_PKT_FLAG_KEY;
        sample->stream_index = track->st->index;

        ret = avpriv_packet_list_put(out, sample, NULL, 0);
        if (ret < 0)
            goto end;

        av_packet_free(&sample);
    }

    ret = 0;

end:
    av_packet_free(&sample);
    for (int i = 0; i < nb_cues; i++)
        av_packet_free(&cues[i].pkt);
    av_freep(&cues);
    av_freep(&times);

    if (ret < 0)
        avpriv_packet_list_free(out);

    return ret;
}

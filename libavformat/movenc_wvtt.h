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

#ifndef AVFORMAT_MOVENC_WVTT_H
#define AVFORMAT_MOVENC_WVTT_H

#include "avformat.h"
#include "movenc.h"
#include "libavcodec/packet_internal.h"

/**
 * Write the WebVTTConfigurationBox ('vttC') of a WVTTSampleEntry.
 */
void ff_mov_write_wvtt_config(AVIOContext *pb, MOVTrack *track);

/**
 * Turn the WebVTT cues queued for a track into the samples of the fragment
 * currently being written, as defined by ISO/IEC 14496-30.
 *
 * The cues of the queue are flattened into consecutive, non overlapping
 * samples: every point in time where a cue starts or ends begins a new sample,
 * a sample carries one 'vttc' box per cue active during it, and stretches of
 * time with no cue at all are covered by a sample carrying a single 'vtte'
 * box. Cues reaching over the end of the fragment are cut and their remainder
 * is put back into the queue for the next fragment.
 *
 * @param out  packet list the generated samples are appended to, in
 *             presentation order
 */
int ff_mov_generate_wvtt_samples(AVFormatContext *s, MOVTrack *track,
                                 PacketList *out);

#endif /* AVFORMAT_MOVENC_WVTT_H */

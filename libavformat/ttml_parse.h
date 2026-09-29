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

#ifndef AVFORMAT_TTML_PARSE_H
#define AVFORMAT_TTML_PARSE_H

#include <stdint.h>

#include "libavcodec/codec_par.h"
#include "libavutil/dict.h"

#define TTML_NS      "http://www.w3.org/ns/ttml"

/**
 * Called for each <p> of a document that has a start time.
 *
 * @param text  the inner XML of the paragraph, wrapped in a <span> carrying
 *              its style and region if it has any. Not NUL terminated
 *              beyond len.
 * @param begin start in ms, relative to the timeline of the document
 * @param end   end in ms, or AV_NOPTS_VALUE if the document does not give one
 * @param line  line of the paragraph in the document
 * @return 0 or a negative AVERROR code, which stops the parsing
 */
typedef int (*FFTTMLParagraphCB)(void *opaque, const char *text, int len,
                                 int64_t begin, int64_t end, int64_t line);

/**
 * Parse a TTML document.
 *
 * @param par      if not NULL, its extradata is set to the <tt> attributes
 *                 and <head> in the format understood by the ttml muxer
 * @param metadata if not NULL, receives the language of the document
 * @param cb       if not NULL, called for each paragraph in document order
 */
int ff_ttml_parse(void *log_ctx, const uint8_t *buf, int size,
                  AVCodecParameters *par, AVDictionary **metadata,
                  FFTTMLParagraphCB cb, void *opaque);

#endif /* AVFORMAT_TTML_PARSE_H */

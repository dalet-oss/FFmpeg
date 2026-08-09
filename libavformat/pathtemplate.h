/*
 * Output path templates shared by the HLS and DASH muxers
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

#ifndef AVFORMAT_PATHTEMPLATE_H
#define AVFORMAT_PATHTEMPLATE_H

#include "libavutil/bprint.h"

enum FFPathTemplateType {
    FF_PATH_TEMPLATE_VIDEO,
    FF_PATH_TEMPLATE_AUDIO,
    FF_PATH_TEMPLATE_SUBTITLE,
    FF_PATH_TEMPLATE_OTHER,     ///< selects no %[...] branch
};

typedef struct FFPathTemplateContext {
    enum FFPathTemplateType type;
    const void *opaque;
    /** Appends the expansion of %v; if NULL, %v is copied verbatim. */
    void (*expand_v)(const void *opaque, AVBPrint *buf);
    /** Appends the expansion of %{name}; unknown names append nothing. */
    void (*expand_named)(const void *opaque, const char *name, int name_len,
                         AVBPrint *buf);
} FFPathTemplateContext;

/**
 * Expand a path template.
 *
 * Handles %% (literal '%'), %v, %{name} and %[v:...|a:...|s:...] blocks
 * (the branch matching tc->type is expanded, %v and %{name} inside it; no
 * branch match gives an empty string). Other %X sequences are copied
 * verbatim for later expansion passes.
 *
 * @param out set to a newly allocated string on success
 * @return 0 on success, AVERROR(EINVAL) on an unclosed %[ block
 */
int ff_path_template_expand(const char *tmpl, char **out,
                            const FFPathTemplateContext *tc);

/**
 * Create the parent directory of a local file path, if it has one.
 */
int ff_path_template_mkdir_parent(const char *path);

#endif /* AVFORMAT_PATHTEMPLATE_H */

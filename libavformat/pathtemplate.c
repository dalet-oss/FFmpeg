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

#include <errno.h>
#include <string.h>

#include "libavutil/avstring.h"
#include "libavutil/error.h"
#include "libavutil/mem.h"

#include "internal.h"
#include "pathtemplate.h"

/* Expands %%, %v and %{...} in p[0..len). With allow_blocks, also expands
 * %[...] blocks (top level only, no nesting). */
static int expand_section(const char *p, size_t len,
                          const FFPathTemplateContext *tc, AVBPrint *buf,
                          int allow_blocks);

/* Expands a %[v:...|a:...|s:...] block. p points just after the opening '%['.
 * Sets *consumed to the number of bytes from p up to and including the ']'. */
static int expand_conditional_block(const char *p,
                                    const FFPathTemplateContext *tc,
                                    AVBPrint *buf, size_t *consumed)
{
    const char *end = strchr(p, ']');
    const char *section = p;

    if (!end) {
        *consumed = strlen(p);
        return AVERROR(EINVAL);
    }
    *consumed = end - p + 1;

    while (section < end) {
        const char *bar = memchr(section, '|', end - section);
        const char *seg_end = bar ? bar : end;
        const char *colon = memchr(section, ':', seg_end - section);
        if (colon && colon - section == 1) {
            char key = section[0];
            if ((key == 'v' && tc->type == FF_PATH_TEMPLATE_VIDEO) ||
                (key == 'a' && tc->type == FF_PATH_TEMPLATE_AUDIO) ||
                (key == 's' && tc->type == FF_PATH_TEMPLATE_SUBTITLE))
                return expand_section(colon + 1, seg_end - colon - 1, tc, buf, 0);
        }
        section = seg_end + 1;
    }
    return 0;
}

static int expand_section(const char *p, size_t len,
                          const FFPathTemplateContext *tc, AVBPrint *buf,
                          int allow_blocks)
{
    size_t k = 0;

    while (k < len) {
        if (p[k] != '%' || k + 1 >= len) {
            av_bprint_chars(buf, p[k++], 1);
            continue;
        }
        if (p[k + 1] == '%') {
            av_bprint_chars(buf, '%', 1);
            k += 2;
        } else if (p[k + 1] == 'v' && tc->expand_v) {
            tc->expand_v(tc->opaque, buf);
            k += 2;
        } else if (p[k + 1] == '{') {
            const char *name = p + k + 2;
            const char *close = memchr(name, '}', len - k - 2);
            if (close) {
                if (tc->expand_named)
                    tc->expand_named(tc->opaque, name, close - name, buf);
                k += 2 + (close - name) + 1;
            } else {
                av_bprint_chars(buf, p[k++], 1);
            }
        } else if (p[k + 1] == '[' && allow_blocks) {
            size_t consumed = 0;
            int ret = expand_conditional_block(p + k + 2, tc, buf, &consumed);
            if (ret < 0)
                return ret;
            k += 2 + consumed;
        } else {
            av_bprint_chars(buf, p[k++], 1);
        }
    }
    return 0;
}

int ff_path_template_expand(const char *tmpl, char **out,
                            const FFPathTemplateContext *tc)
{
    AVBPrint buf;
    int ret;

    av_bprint_init(&buf, 0, AV_BPRINT_SIZE_UNLIMITED);
    ret = expand_section(tmpl, strlen(tmpl), tc, &buf, 1);
    if (ret < 0) {
        av_bprint_finalize(&buf, NULL);
        return ret;
    }
    return av_bprint_finalize(&buf, out);
}

int ff_path_template_mkdir_parent(const char *path)
{
    char *dir_copy = av_strdup(path);
    const char *dir;
    int ret = 0;

    if (!dir_copy)
        return AVERROR(ENOMEM);
    dir = av_dirname(dir_copy);
    if (strcmp(dir, ".") && strcmp(dir, path)) {
        if (ff_mkdir_p(dir) == -1 && errno != EEXIST)
            ret = AVERROR(errno);
    }
    av_free(dir_copy);
    return ret;
}

#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"
#include "libavutil/frame.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "thread.h"

#include <kakadusdk_c_wrapper.h>

// pix_fmts with lower bpp have to be listed before
// similar pix_fmts with higher bpp.
#define RGB_PIXEL_FORMATS  AV_PIX_FMT_RGB24, AV_PIX_FMT_RGBA, \
                           AV_PIX_FMT_RGB48, AV_PIX_FMT_RGBA64

#define GRAY_PIXEL_FORMATS AV_PIX_FMT_GRAY8, AV_PIX_FMT_YA8, \
                           AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, \
                           AV_PIX_FMT_GRAY16, AV_PIX_FMT_YA16

#define YUV_PIXEL_FORMATS  AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUVA420P, \
                           AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUVA422P, \
                           AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUVA444P, \
                           AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9, \
                           AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA444P9, \
                           AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10, \
                           AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA444P10, \
                           AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12, \
                           AV_PIX_FMT_YUV420P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV444P14, \
                           AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16, \
                           AV_PIX_FMT_YUVA420P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA444P16

#define XYZ_PIXEL_FORMATS  AV_PIX_FMT_XYZ12

static const enum AVPixelFormat libkakaduc_rgb_pix_fmts[]  = {
        RGB_PIXEL_FORMATS
};
static const enum AVPixelFormat libkakaduc_gray_pix_fmts[] = {
        GRAY_PIXEL_FORMATS
};
static const enum AVPixelFormat libkakaduc_yuv_pix_fmts[]  = {
        YUV_PIXEL_FORMATS
};
static const enum AVPixelFormat libkakaduc_all_pix_fmts[]  = {
        RGB_PIXEL_FORMATS, GRAY_PIXEL_FORMATS, YUV_PIXEL_FORMATS, XYZ_PIXEL_FORMATS
};

typedef struct LibKakaducContext {
    AVClass *class;
    ffmpeg_log_callbacks log_callbacks;
    void *kakaduc_handle;
} LibKakaducContext;

static void debug_callback(const char *msg, void *data)
{
    av_log(data, AV_LOG_DEBUG, "%s\n", msg);
}

static void info_callback(const char *msg, void *data)
{
    av_log(data, AV_LOG_INFO, "%s\n", msg);
}

static void warning_callback(const char *msg, void *data)
{
    av_log(data, AV_LOG_WARNING, "%s\n", msg);
}

static void error_callback(const char *msg, void *data)
{
    av_log(data, AV_LOG_ERROR, "%s\n", msg);
}

static inline int libkakaduc_ispacked(enum AVPixelFormat pix_fmt)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    int i, component_plane;

    if (pix_fmt == AV_PIX_FMT_GRAY16)
        return 0;

    component_plane = desc->comp[0].plane;
    for (i = 1; i < desc->nb_components; i++)
        if (component_plane != desc->comp[i].plane)
            return 0;
    return 1;
}

static inline int libkakaduc_supports_pix_fmt(AVCodecContext *avctx, const kakadu_info *info, enum AVPixelFormat pix_fmt)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    int supported = 1;
    int is_packed, i, bit_depth = -1;

    if (desc->nb_components != info->num_components) {
        av_log(avctx, AV_LOG_DEBUG, "Skipping pix_fmt due to component count mismatch: %s\n",
               av_get_pix_fmt_name(pix_fmt));
        return 0;
    }

    switch (desc->nb_components) {
        case 4:
            supported = supported &&
                    desc->comp[3].depth >= info->components[3].bit_depth &&
                    1 == info->components[3].subsampling_x &&
                    1 == info->components[3].subsampling_y;
        case 3:
            supported = supported &&
                    desc->comp[2].depth >= info->components[2].bit_depth &&
                    (1 << desc->log2_chroma_w) == info->components[2].subsampling_x &&
                    (1 << desc->log2_chroma_h) == info->components[2].subsampling_y;
        case 2:
            supported = supported &&
                    desc->comp[1].depth >= info->components[1].bit_depth &&
                    (1 << desc->log2_chroma_w) == info->components[1].subsampling_x &&
                    (1 << desc->log2_chroma_h) == info->components[1].subsampling_y;
        case 1:
            supported = supported &&
                    desc->comp[0].depth >= info->components[0].bit_depth &&
                    1 == info->components[0].subsampling_x &&
                    1 == info->components[0].subsampling_y;
        default:
            break;
    }

    if (!supported) {
        av_log(avctx, AV_LOG_DEBUG, "Skipping pix_fmt due to bit-depth or sub-sampling mismatch: %s\n",
               av_get_pix_fmt_name(pix_fmt));

        return 0;
    }

    is_packed = libkakaduc_ispacked(pix_fmt);

    if (is_packed && (desc->log2_chroma_w > 0 || desc->log2_chroma_h > 0)) {
        av_log(avctx, AV_LOG_DEBUG, "Skipping pix_fmt as packed formats with subsampling are not supported: %s\n",
               av_get_pix_fmt_name(pix_fmt));

        return 0;
    }

    for (i = 0; i < desc->nb_components; i++) {
        if (desc->comp[i].depth > 32) {
            av_log(avctx, AV_LOG_DEBUG, "Skipping pix_fmt as unsupported bit depth %d for component %d: %s\n",
                   desc->comp[i].depth, i, av_get_pix_fmt_name(pix_fmt));

            return 0;
        }
        if (bit_depth == -1) {
            bit_depth = desc->comp[i].depth;
        } else if (bit_depth != desc->comp[i].depth) {
            av_log(avctx, AV_LOG_DEBUG, "Skipping pix_fmt as mixed bit depths are not supported: %s\n",
                   av_get_pix_fmt_name(pix_fmt));
            return 0;
        }
    }

    return supported;
}

static inline enum AVPixelFormat libkakaduc_choose_pix_fmt(AVCodecContext *avctx, const kakadu_info *info) {
    int index;
    const enum AVPixelFormat *possible_fmts = NULL;
    int possible_fmts_nb = 0;

    switch (info->colorspace) {
        case KAKADUC_COLORSPACE_RGB:
            possible_fmts = libkakaduc_rgb_pix_fmts;
            possible_fmts_nb = FF_ARRAY_ELEMS(libkakaduc_rgb_pix_fmts);
            break;
        case KAKADUC_COLORSPACE_GRAYSCALE:
            possible_fmts = libkakaduc_gray_pix_fmts;
            possible_fmts_nb = FF_ARRAY_ELEMS(libkakaduc_gray_pix_fmts);
            break;
        case KAKADUC_COLORSPACE_YCC:
            possible_fmts = libkakaduc_yuv_pix_fmts;
            possible_fmts_nb = FF_ARRAY_ELEMS(libkakaduc_yuv_pix_fmts);
            break;
        default:
            possible_fmts = libkakaduc_all_pix_fmts;
            possible_fmts_nb = FF_ARRAY_ELEMS(libkakaduc_all_pix_fmts);
            break;
    }

    for (index = 0; index < possible_fmts_nb; ++index)
        if (libkakaduc_supports_pix_fmt(avctx, info, possible_fmts[index])) {
            return possible_fmts[index];
        }

    return AV_PIX_FMT_NONE;
}

static av_cold int libkakaduc_decode_init(AVCodecContext *avctx)
{
    LibKakaducContext *ctx = avctx->priv_data;

    ctx->log_callbacks.LogDebug = debug_callback;
    ctx->log_callbacks.LogInfo = info_callback;
    ctx->log_callbacks.LogWarning = warning_callback;
    ctx->log_callbacks.LogError = error_callback;
    ctx->kakaduc_handle = kakadusdk_init(&ctx->log_callbacks, avctx);

    if (!ctx->kakaduc_handle) {
        return AVERROR_EXTERNAL;
    }

    return 0;
}

static av_cold int libkakaduc_decode_cleanup(AVCodecContext *avctx)
{
    LibKakaducContext *ctx = avctx->priv_data;

    if (ctx->kakaduc_handle) {
        kakadusdk_cleanup(ctx->kakaduc_handle);
        ctx->kakaduc_handle = NULL;
    }

    return 0;
}

static int libkakaduc_decode_frame(AVCodecContext *avctx, AVFrame *picture,
                                    int *got_frame, AVPacket *avpkt)
{
    int i, width, height, ret;
    const AVPixFmtDescriptor *desc;
    int is_packed = 0;

    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    LibKakaducContext *ctx = avctx->priv_data;

    kakadu_info *info = NULL;
    *got_frame = 0;

    info = kakadusdk_open(ctx->kakaduc_handle, buf, buf_size);

    if (!info) {
        av_log(avctx, AV_LOG_ERROR, "Error initializing decoder\n");
        ret = AVERROR_EXTERNAL;
        goto done;
    }

    // use largest component dimensions as frame size
    width  = info->components[0].width;
    height = info->components[0].height;
    for (i = 1; i < info->num_components; i++) {
        if (info->components[i].width > width)
            width = info->components[i].width;
        if (info->components[i].height > height)
            height = info->components[i].height;
    }

    ret = ff_set_dimensions(avctx, width, height);
    if (ret < 0)
        goto done;

    if (avctx->pix_fmt != AV_PIX_FMT_NONE)
        if (!libkakaduc_supports_pix_fmt(avctx, info, avctx->pix_fmt))
            avctx->pix_fmt = AV_PIX_FMT_NONE;

    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        avctx->pix_fmt = libkakaduc_choose_pix_fmt(avctx, info);
        avctx->color_range = AVCOL_RANGE_JPEG;
        if (info->colorspace == KAKADUC_COLORSPACE_RGB)
            avctx->colorspace = AVCOL_SPC_RGB;
        else if (info->colorspace == KAKADUC_COLORSPACE_YCC)
            avctx->colorspace = AVCOL_SPC_BT709;
        else
            avctx->colorspace = AVCOL_SPC_UNSPECIFIED;
    }

    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        av_log(avctx, AV_LOG_ERROR, "Unable to determine pixel format\n");
        ret = AVERROR_UNKNOWN;
        goto done;
    }

    desc = av_pix_fmt_desc_get(avctx->pix_fmt);
    is_packed = libkakaduc_ispacked(avctx->pix_fmt);

    av_log(avctx, AV_LOG_TRACE, "Chose pixel format %s, packed: %d\n",
           av_get_pix_fmt_name(avctx->pix_fmt), is_packed);
    av_log(avctx, AV_LOG_TRACE, "Set colorspace %d\n", avctx->colorspace);

    avctx->bits_per_raw_sample = desc->comp[0].depth;

    if ((ret = ff_thread_get_buffer(avctx, picture, 0)) < 0)
        goto done;

    for (i = 0; i < desc->nb_components; i++) {
        av_log(avctx, AV_LOG_TRACE, "picture->buf[%d]->size -> %ld\n", i, picture->buf[i]->size);
        av_log(avctx, AV_LOG_TRACE, "picture->linesize[%d] -> %d\n", i, picture->linesize[i]);
    }

    ret = kakadusdk_decode(ctx->kakaduc_handle, desc->comp[0].depth, is_packed, picture->data, picture->linesize);

    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Error decoding codestream\n");
        ret = AVERROR_EXTERNAL;
        goto done;
    }

    *got_frame = 1;

    picture->pict_type = AV_PICTURE_TYPE_I;
    picture->flags |= AV_FRAME_FLAG_KEY;
    ret = buf_size;

    done:

    kakadusdk_close(ctx->kakaduc_handle);

    return ret;
}

static const AVClass kakaduc_class = {
    .class_name = "kakaduc",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_libkakaduc_decoder = {
    .p.name         = "libkakaduc",
    CODEC_LONG_NAME("Kakadu C JPEG 2000"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_JPEG2000,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS,
    .p.priv_class   = &kakaduc_class,
    .p.wrapper_name = "libkakaduc",
    .p.pix_fmts     = (const enum AVPixelFormat[]) { RGB_PIXEL_FORMATS, AV_PIX_FMT_GRAY8, GRAY_PIXEL_FORMATS, YUV_PIXEL_FORMATS, XYZ_PIXEL_FORMATS, AV_PIX_FMT_NONE },
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .priv_data_size = sizeof(LibKakaducContext),
    .init           = libkakaduc_decode_init,
    .close          = libkakaduc_decode_cleanup,
    FF_CODEC_DECODE_CB(libkakaduc_decode_frame),
};

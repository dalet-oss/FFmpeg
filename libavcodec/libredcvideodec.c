#include "libavutil/common.h"
#include "avcodec.h"
#include "decode.h"
#include "codec_internal.h"

#include "libavformat/libredc.h"

static av_cold int libredc_video_decode_init(AVCodecContext *avctx) {

    LibRedcVideoContext *ctx = avctx->priv_data;

    av_log(ctx, AV_LOG_TRACE, "libredc_video_decode_init\n");

    ctx->current_frame_offset = 0;
    ctx->current_time_offset = 0;

    avctx->pix_fmt = avctx->codec->pix_fmts[0];
    avctx->bits_per_raw_sample = 8;

    return 0;
}

static av_cold int libredc_video_decode_cleanup(AVCodecContext *avctx) {


    LibRedcVideoContext *ctx = avctx->priv_data;

    av_log(ctx, AV_LOG_TRACE, "libredc_video_decode_cleanup\n");

    return 0;
}

static int libredc_video_decode_frame(AVCodecContext *avctx, AVFrame *frame, int *got_frame_ptr, AVPacket *avpkt) {

    int buf_size = avpkt->size;
    int width = avctx->width;
    int height = avctx->height;
    LibRedcAudioContext *ctx = avctx->priv_data;
    AVBufferRef *buf_ref;
    int n, ret;

    av_log(ctx, AV_LOG_TRACE, "libredc_video_decode_frame\n");

    if (avctx->codec_id != avctx->codec->id) {
        av_log(avctx, AV_LOG_ERROR, "codec ids mismatch\n");
        return AVERROR(EINVAL);
    }

    ret = ff_set_dimensions(avctx, width, height);
    if (ret < 0)
        return ret;

    if (avctx->pix_fmt == AV_PIX_FMT_NONE)
        avctx->pix_fmt = AV_PIX_FMT_BGR24;

    if (avctx->pix_fmt != AV_PIX_FMT_BGR24) {
        av_log(avctx, AV_LOG_ERROR, "Unable to determine pixel format\n");
        return AVERROR_UNKNOWN;
    }

    // three channels (BGR) in 8-bit interleaved values
    n = width * height * 3;

    if (n && buf_size % n) {
        if (buf_size < n) {
            av_log(avctx, AV_LOG_ERROR,
                   "Invalid packet, data has size %d but at least a size of %d was expected\n",
                   buf_size, n);
            return AVERROR_INVALIDDATA;
        }
        else {
            if (buf_size % n) {
                av_log(avctx, AV_LOG_WARNING,
                   "Packet size %d is not a multiple of frame size %d, trimming extra data\n",
                   buf_size, n);
            }
            buf_size -= buf_size % n;
        }
    }

    // copy the pointer of the packet buffer to the pointer of the frame buffer
    buf_ref = av_buffer_ref(avpkt->buf);
    if (!buf_ref) {
        return AVERROR(ENOMEM);
    }

    frame->buf[0] = buf_ref;
    frame->data[0] = avpkt->data;
    frame->linesize[0] = width * 3; // three channels (BGR) in 8-bit interleaved values
    frame->format = avctx->pix_fmt;
    frame->pts = avpkt->pts;
    frame->width = avctx->width;
    frame->height = avctx->height;
    frame->color_range = AVCOL_RANGE_JPEG;

    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags |= AV_FRAME_FLAG_KEY;

    ret = buf_size;

    *got_frame_ptr = 1;

    return ret;
}

static const AVClass redcvideo_class = {
    .class_name = "redc_video",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_LIBREDC
const FFCodec ff_libredcvideo_decoder = {
    .p.name         = "libredc_video",
    CODEC_LONG_NAME("Red C Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_REDCVIDEO,
    .p.priv_class   = &redcvideo_class,
    .p.wrapper_name = "libredcvideo",
    .p.pix_fmts     = (const enum AVPixelFormat[]) { AV_PIX_FMT_BGR24, AV_PIX_FMT_NONE },
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .priv_data_size = sizeof(LibRedcVideoContext),
    .init           = libredc_video_decode_init,
    .close          = libredc_video_decode_cleanup,
    FF_CODEC_DECODE_CB(libredc_video_decode_frame)
};
#endif

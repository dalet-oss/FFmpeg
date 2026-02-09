#include "libavutil/common.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

#include "libavformat/libredc.h"

static av_cold int libredc_audio_decode_init(AVCodecContext *avctx) {
    LibRedcAudioContext *ctx = avctx->priv_data;

    av_log(ctx, AV_LOG_TRACE, "libredc_audio_decode_init\n");

    ctx->current_sample_offset = 0;

    avctx->sample_fmt = avctx->codec->sample_fmts[0];

    return 0;
}

static av_cold int libredc_audio_decode_cleanup(AVCodecContext *avctx) {
    LibRedcAudioContext *ctx = avctx->priv_data;

    av_log(ctx, AV_LOG_TRACE, "libredc_audio_decode_cleanup\n");

    return 0;
}

static int libredc_audio_decode_frame(AVCodecContext *avctx, AVFrame *frame, int *got_frame_ptr, AVPacket *avpkt) {

    int buf_size = avpkt->size;
    int channels = avctx->ch_layout.nb_channels;
    LibRedcAudioContext *ctx = avctx->priv_data;
    int n;
    AVBufferRef *buf_ref;

    av_log(ctx, AV_LOG_TRACE, "libredc_audio_decode_frame\n");

    if (channels == 0) {
        av_log(avctx, AV_LOG_ERROR, "Invalid number of channels\n");
        return AVERROR(EINVAL);
    }

    if (avctx->codec_id != avctx->codec->id) {
        av_log(avctx, AV_LOG_ERROR, "codec ids mismatch\n");
        return AVERROR(EINVAL);
    }

    // 24 bit samples are stored in 32 bit containers
    n = channels * 4;

    if (n && buf_size % n) {
        if (buf_size < n) {
            av_log(avctx, AV_LOG_ERROR,
                   "Invalid packet, data has size %d but at least a size of %d was expected\n",
                   buf_size, n);
            return AVERROR_INVALIDDATA;
        } else
            buf_size -= buf_size % n;
    }

    // copy the pointer of the packet buffer to the pointer of the frame buffer
    buf_ref = av_buffer_ref(avpkt->buf);
    if (!buf_ref) {
        return AVERROR(ENOMEM);
    }

    frame->buf[0] = buf_ref;
    frame->data[0] = avpkt->data;
    frame->linesize[0] = buf_size;
    frame->nb_samples = avpkt->duration;
    frame->format = avctx->sample_fmt;
    frame->pts = avpkt->pts;
    frame->ch_layout = avctx->ch_layout;
    frame->sample_rate = avctx->sample_rate;

    av_log(ctx, AV_LOG_TRACE, "frame->sample_fmt %d\n", frame->format);
    av_log(ctx, AV_LOG_TRACE, "frame->linesize[0] %d\n", frame->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "frame->nb_samples %d\n", frame->nb_samples);
    av_log(ctx, AV_LOG_TRACE, "frame->ch_layout.nb_channels %d\n", frame->ch_layout.nb_channels);
    av_log(ctx, AV_LOG_TRACE, "frame->sample_rate %d\n", frame->sample_rate);
    av_log(ctx, AV_LOG_TRACE, "buf_ref %ld\n", buf_ref->size);

    *got_frame_ptr = 1;

    return buf_size;
}

static const AVClass redcaudio_class = {
    .class_name = "redc_audio",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_LIBREDC
const FFCodec ff_libredcaudio_decoder = {
    .p.name         = "libredc_audio",
    CODEC_LONG_NAME("Red C Audio"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_REDCAUDIO,
    .p.priv_class   = &redcaudio_class,
    .p.wrapper_name = "libredcaudio",
    .priv_data_size = sizeof(LibRedcAudioContext),
    .p.sample_fmts  = (const enum AVSampleFormat[]){ AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_NONE },
    .init           = libredc_audio_decode_init,
    .close          = libredc_audio_decode_cleanup,
    FF_CODEC_DECODE_CB(libredc_audio_decode_frame),
};
#endif

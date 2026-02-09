#include "libavutil/intreadwrite.h"
#include "libavutil/dict.h"
#include "libavutil/mathematics.h"
#include "libavutil/channel_layout.h"
#include "libavutil/timecode.h"


#include "avio.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#include "libredc.h"

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

static void *open_callback(const char* utf8Path, void *data)
{
    AVFormatContext *avctx = data;
    LibRedcContext *ctx = avctx->priv_data;
    RedcFileContext *related_redc_file_ctx = NULL;

    av_log(data, AV_LOG_DEBUG, "Open request for URL: %s\n", utf8Path);

    if (!strcmp(utf8Path, ctx->redc_files[0]->path)) {
        av_log(data, AV_LOG_DEBUG, "Open request for original URL: %s, returning existing AVIOContext\n", utf8Path);

        return ctx->redc_files[0]->pb;
    }

    related_redc_file_ctx = av_mallocz(sizeof(RedcFileContext));
    if (!related_redc_file_ctx) {
        av_log(data, AV_LOG_ERROR, "Open request for URL: %s, unable to allocate RedcFileContext\n", utf8Path);

        return NULL;
    }

    related_redc_file_ctx->path = av_strdup(utf8Path);
    if (!related_redc_file_ctx->path) {
        av_log(data, AV_LOG_ERROR, "Open request for URL: %s, unable to allocate path string\n", utf8Path);

        av_free(related_redc_file_ctx);
        return NULL;
    }

    if (avio_open(&related_redc_file_ctx->pb, utf8Path, AVIO_FLAG_READ) < 0) {
        av_log(data, AV_LOG_ERROR, "Open request for URL: %s, unable to open AVIOContext\n", utf8Path);

        av_free(related_redc_file_ctx->path);
        av_free(related_redc_file_ctx);
        return NULL;
    }

    dynarray_add(&ctx->redc_files, &ctx->n_redc_files, related_redc_file_ctx);

    return related_redc_file_ctx->pb;
}

static void close_callback(void* handle, void *data)
{
    AVFormatContext *avctx = data;
    LibRedcContext *ctx = avctx->priv_data;
    int i;

    if (handle == ctx->redc_files[0]->pb) {
        av_log(data, AV_LOG_DEBUG, "Close request for original URL: %s, ignoring\n", ctx->redc_files[0]->path);

        return;
    }

    // find the redc file
    for (i = 1; i < ctx->n_redc_files; i++) {
        if (ctx->redc_files[i]->pb == handle) {
            break;
        }
    }

    if (i >= ctx->n_redc_files) {
        av_log(data, AV_LOG_ERROR, "Close request for unknown handle\n");
        return;
    }

    avio_closep(&ctx->redc_files[i]->pb);
    ctx->redc_files[i]->pb = NULL;

    av_log(data, AV_LOG_DEBUG, "Closed handle for URL: %s\n", ctx->redc_files[i]->path);
}

static unsigned long long filesize_callback(void* handle, void *data)
{
    AVFormatContext *avctx = data;
    LibRedcContext *ctx = avctx->priv_data;
    int i;
    unsigned long long size;

    // find the redc file
    for (i = 0; i < ctx->n_redc_files; i++) {
        if (ctx->redc_files[i]->pb == handle) {
            break;
        }
    }

    if (i >= ctx->n_redc_files) {
        av_log(data, AV_LOG_ERROR, "Filesize request for unknown handle\n");

        return -1;
    }

    size = avio_size(handle);

    av_log(data, AV_LOG_DEBUG, "Filesize URL: %s = %lld\n", ctx->redc_files[i]->path, size);

    return size;
}

static int read_callback(void* outBuffer, size_t bytes, unsigned long long offset, void* handle, void *data)
{
    AVFormatContext *avctx = data;
    LibRedcContext *ctx = avctx->priv_data;
    int i;

    // find the redc file
    for (i = 0; i < ctx->n_redc_files; i++) {
        if (ctx->redc_files[i]->pb == handle) {
            break;
        }
    }

    if (i >= ctx->n_redc_files) {
        av_log(data, AV_LOG_ERROR, "Read request for unknown handle\n");

        return 0;
    }

    av_log(data, AV_LOG_DEBUG, "Read %s\n", ctx->redc_files[i]->path);

    // check if offset is greater than INT64_MAX
    if (offset > INT64_MAX) {
        av_log(data, AV_LOG_ERROR, "Read request for offset %llu exceeds INT64_MAX\n", offset);

        return AVERROR(EINVAL);
    }

    // check if bytes is greater than INT_MAX
    if (bytes > INT_MAX) {
        av_log(data, AV_LOG_ERROR, "Read request for %zu bytes exceeds INT_MAX\n", bytes);

        return AVERROR(EINVAL);
    }

    if (avio_seek(handle, (int64_t)offset, SEEK_SET) != offset) {
        av_log(data, AV_LOG_ERROR, "Read request seek to offset %llu failed\n", offset);

        return AVERROR(EIO);
    }

    i = avio_read(handle, outBuffer, (int)bytes);

    if (i != (int)bytes) {
        av_log(data, AV_LOG_DEBUG, "Read request for %zu bytes returned %d\n", bytes, i);

        return 1;
    }

    av_log(data, AV_LOG_DEBUG, "Read %zu bytes\n", bytes);

    return 0;
}

static int redc_read_header(AVFormatContext *avctx) {

    LibRedcContext *ctx = avctx->priv_data;
    RedcFileContext *original_redc_file_ctx = NULL;
    red_info* info = NULL;
    AVStream *video_st;
    AVStream *audio_st;
    AVCodecParameters *audio_par;
    AVCodecParameters *video_par;
    LibRedcVideoContext *video_ctx;
    LibRedcAudioContext *audio_ctx;

    original_redc_file_ctx = av_mallocz(sizeof(RedcFileContext));
    if (!original_redc_file_ctx) {
        return AVERROR(ENOMEM);
    }

    original_redc_file_ctx->path = av_strdup(avctx->url);
    if (!original_redc_file_ctx->path) {
        av_free(original_redc_file_ctx);
        return AVERROR(ENOMEM);
    }

    original_redc_file_ctx->pb = avctx->pb;

    ctx->redc_files = NULL;
    ctx->n_redc_files = 0;

    av_dynarray_add(&ctx->redc_files, &ctx->n_redc_files, original_redc_file_ctx);
    if (ctx->n_redc_files == 0)
        return AVERROR(ENOMEM);

    ctx->io_callbacks.Open = open_callback;
    ctx->io_callbacks.Close = close_callback;
    ctx->io_callbacks.Filesize = filesize_callback;
    ctx->io_callbacks.Read = read_callback;

    ctx->log_callbacks.LogDebug = debug_callback;
    ctx->log_callbacks.LogInfo = info_callback;
    ctx->log_callbacks.LogWarning = warning_callback;
    ctx->log_callbacks.LogError = error_callback;

    ctx->redc_handle = redsdk_init(&ctx->io_callbacks, &ctx->log_callbacks, avctx);

    if (!ctx->redc_handle) {
        return AVERROR_EXTERNAL;
    }

    info = redsdk_open(ctx->redc_handle, avctx->url);

    if (!info)  {
        return AVERROR_EXTERNAL;
    }

    if (info->start_absolute_timecode) {
        av_dict_set(&avctx->metadata, "timecode", info->start_absolute_timecode, 0);
    }

    video_st = avformat_new_stream(avctx, NULL);

    if (!video_st) {
        return AVERROR(ENOMEM);
    }

    video_st->priv_data = av_malloc(sizeof(LibRedcVideoContext));

    if (!video_st->priv_data) {
        return AVERROR(ENOMEM);
    }

    video_ctx = video_st->priv_data;
    video_ctx->redc_handle = ctx->redc_handle;
    video_ctx->frame_count = info->video_frame_count;
    video_ctx->current_frame_offset = 0;
    video_ctx->current_time_offset = 0;

    video_par = video_st->codecpar;

    video_par->codec_type = AVMEDIA_TYPE_VIDEO;
    video_par->codec_id = AV_CODEC_ID_REDCVIDEO;
    video_par->format = AV_PIX_FMT_BGR24;
    video_par->width = info->width;
    video_par->height = info->height;
    video_par->framerate = (AVRational){(int)(info->video_framerate * 1000.0), 1000 };
    video_par->color_range = AVCOL_RANGE_JPEG;

    video_st->nb_frames = info->video_frame_count;

    video_st->id = 0;
    video_st->start_time = 0;
    video_st->avg_frame_rate = (AVRational){(int)(info->video_framerate * 1000.0), 1000 };
    video_st->r_frame_rate = (AVRational){(int)(info->video_framerate * 1000.0), 1000 };
    avpriv_set_pts_info(video_st, 64, video_st->avg_frame_rate.den, video_st->avg_frame_rate.num);
    video_st->duration = video_st->nb_frames;

    if (info->audio_channel_count > 0) {
        audio_st = avformat_new_stream(avctx, NULL);

        if (!audio_st) {
            return AVERROR(ENOMEM);
        }

        audio_st->priv_data = av_malloc(sizeof(LibRedcAudioContext));

        if (!audio_st->priv_data) {
            return AVERROR(ENOMEM);
        }

        audio_ctx = audio_st->priv_data;
        audio_ctx->redc_handle = ctx->redc_handle;
        audio_ctx->sample_count = info->audio_sample_count;
        audio_ctx->current_sample_offset = 0;
        audio_ctx->current_time_offset = 0;

        audio_par = audio_st->codecpar;

        audio_par->codec_type = AVMEDIA_TYPE_AUDIO;
        audio_par->codec_id = AV_CODEC_ID_REDCAUDIO;
        audio_par->format = AV_SAMPLE_FMT_S32;
        audio_par->sample_rate = info->audio_sample_rate;
        audio_par->bits_per_raw_sample = info->audio_sample_size;
        audio_par->block_align = audio_par->bits_per_coded_sample * audio_par->ch_layout.nb_channels / 8;
        audio_par->bits_per_coded_sample = info->audio_sample_size;

        switch (info->audio_channel_count) {
            case 1:
                audio_par->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
                break;
            case 2:
                audio_par->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
                break;
            case 4:
                audio_par->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_4POINT0;
                break;
            default:
                av_log(avctx, AV_LOG_ERROR, "Unsupported audio channel count: %ld\n", info->audio_channel_count);
                return AVERROR_EXTERNAL;
        }

        audio_st->id = 1;
        audio_st->start_time = 0;
        avpriv_set_pts_info(audio_st, 64, 1, audio_par->sample_rate);
        audio_st->duration = info->audio_sample_count;
    }

    return 0;
}

static int redc_read_video_packet(AVFormatContext *s, AVPacket *pkt) {
    int ret, size;
    AVStream *video = s->streams[0];
    LibRedcContext *ctx = s->priv_data;
    LibRedcVideoContext *video_ctx = video->priv_data;

    av_log(ctx, AV_LOG_TRACE, "redc_read_video_packet\n");

    if (video_ctx->current_frame_offset >= video_ctx->frame_count) {
        av_log(ctx, AV_LOG_DEBUG, "returning EOF for video\n");
        return AVERROR_EOF;
    }

    // three channels (BGR) in 8-bit interleaved values
    size = video->codecpar->width * video->codecpar->height * 3;

    ret = av_new_packet(pkt, size);
    if (ret < 0)
        return ret;

    if (!red_sdk_decode_video(ctx->redc_handle, video_ctx->current_frame_offset, pkt->data, size)) {
        av_log(ctx, AV_LOG_ERROR, "failed to decode video frame\n");

        av_packet_unref(pkt);

        return AVERROR_EXTERNAL;
    }

    pkt->stream_index = video->index;
    pkt->pts = (int64_t)video_ctx->current_frame_offset;
    pkt->duration = 1;
    pkt->size = size;

    video_ctx->current_frame_offset += 1;
    video_ctx->current_time_offset = av_rescale_q(
        (int64_t)video_ctx->current_frame_offset,
        video->codecpar->framerate,
        (AVRational){1, AV_TIME_BASE}
    );

    return 0;

}

static int redc_read_audio_packet(AVFormatContext *s, AVPacket *pkt) {
    int ret, size;
    size_t sample_count = 1024;
    AVStream *audio = s->streams[1];
    LibRedcContext *ctx = s->priv_data;
    LibRedcAudioContext *audio_ctx = audio->priv_data;

    av_log(ctx, AV_LOG_TRACE, "redc_read_audio_packet\n");

    if (audio_ctx->current_sample_offset >= audio_ctx->sample_count) {
        av_log(ctx, AV_LOG_DEBUG, "returning EOF for audio\n");
        return AVERROR_EOF;
    }

    // 32 bit sample storage of 24 bit samples
    size = sample_count * audio->codecpar->ch_layout.nb_channels * 4;

    ret = av_new_packet(pkt, size);
    if (ret < 0)
        return ret;

    if (!red_sdk_decode_audio(ctx->redc_handle, audio_ctx->current_sample_offset, &sample_count, pkt->data, size)) {
        av_log(ctx, AV_LOG_ERROR, "failed to decode audio samples\n");

        av_packet_unref(pkt);

        return AVERROR_EXTERNAL;
    }

    pkt->stream_index = audio->index;
    pkt->pts = (int64_t)audio_ctx->current_sample_offset;
    pkt->duration = (int)sample_count;
    pkt->size = sample_count * audio->codecpar->ch_layout.nb_channels * 4;

    audio_ctx->current_sample_offset += sample_count;
    audio_ctx->current_time_offset = av_rescale_q(
        (int64_t)audio_ctx->current_sample_offset,
        (AVRational){1, audio->codecpar->sample_rate},
        (AVRational){1, AV_TIME_BASE}
    );

    return 0;
}

static int redc_read_packet(AVFormatContext *s, AVPacket *pkt) {
    av_log(s, AV_LOG_TRACE, "redc_read_packet\n");

    // video only
    if (s->nb_streams == 1) {
        if (s->streams[0]->discard != AVDISCARD_ALL) {
            return redc_read_video_packet(s, pkt);
        }

        return AVERROR_EOF;
    }

    // video + audio
    if ((s->streams[0]->discard != AVDISCARD_ALL) && (s->streams[1]->discard != AVDISCARD_ALL)) {

        AVStream *video = s->streams[0];
        AVStream *audio = s->streams[1];
        LibRedcVideoContext *video_ctx = video->priv_data;
        LibRedcAudioContext *audio_ctx = audio->priv_data;

        // decide whether to read video or audio packet based on current time offsets
        if (video_ctx->current_time_offset <= audio_ctx->current_time_offset) {
            return redc_read_video_packet(s, pkt);
        }
        else {
            return redc_read_audio_packet(s, pkt);
        }
    }
    if (s->streams[0]->discard != AVDISCARD_ALL) {
        return redc_read_video_packet(s, pkt);
    }
    if (s->streams[1]->discard != AVDISCARD_ALL) {
        return redc_read_audio_packet(s, pkt);
    }

    return AVERROR_EOF;
}

static int redc_probe(const AVProbeData *p) {

    av_log(NULL, AV_LOG_TRACE, "redc_probe\n");

    if (AV_RL32(p->buf + 4) == MKTAG('R','E','D','2')) {
        av_log(NULL, AV_LOG_TRACE, "redc_probe => AVPROBE_SCORE_MAX\n");
        return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int redc_seek(AVFormatContext *s, int stream_index, int64_t sample_time, int flags) {

    LibRedcContext *ctx = s->priv_data;

    av_log(ctx, AV_LOG_DEBUG, "redc_seek\n");

    return -1;
}

static int redc_close(AVFormatContext *s)
{
    LibRedcContext *ctx = s->priv_data;
    int i;

    if (ctx->redc_handle) {
        redsdk_close(ctx->redc_handle);
        redsdk_cleanup(ctx->redc_handle);
        ctx->redc_handle = NULL;
    }

    for (i = 0; i < ctx->n_redc_files; i++) {
        av_freep(&ctx->redc_files[i]->path);
    }

    av_freep(&ctx->redc_files);
    ctx->redc_files = NULL;

    return 0;
}

static const AVClass redc_class = {
    .class_name = "redc",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_libredc_demuxer = {
    .p.name         = "libredc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("R3D C"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(LibRedcContext),
    .p.priv_class   = &redc_class,
    .read_probe     = redc_probe,
    .read_header    = redc_read_header,
    .read_packet    = redc_read_packet,
    .read_seek      = redc_seek,
    .read_close     = redc_close
};

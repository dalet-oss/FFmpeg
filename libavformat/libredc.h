#include "libavutil/intreadwrite.h"
#include "libavutil/dict.h"
#include "libavutil/mathematics.h"

#include "avio.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#include "redsdk_c_wrapper.h"

typedef struct RedcFileContext {
    char *path;
    AVIOContext *pb;
} RedcFileContext;

typedef struct LibRedcContext {
    AVClass *class;
    int n_redc_files;
    struct RedcFileContext **redc_files;
    AVDictionary *avio_opts;

    ffmpeg_log_callbacks log_callbacks;
    ffmpeg_io_callbacks io_callbacks;
    void *redc_handle;
} LibRedcContext;

typedef struct LibRedcVideoContext {
    AVClass *class;
    void *redc_handle;
    unsigned long long frame_count;
    unsigned long long current_frame_offset;
    unsigned long long current_time_offset;
} LibRedcVideoContext;

typedef struct LibRedcAudioContext {
    AVClass *class;
    void *redc_handle;
    unsigned long long sample_count;
    unsigned long long current_sample_offset;
    unsigned long long current_time_offset;
} LibRedcAudioContext;

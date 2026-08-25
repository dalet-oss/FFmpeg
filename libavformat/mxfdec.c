/*
 * MXF demuxer.
 * Copyright (c) 2006 SmartJog S.A., Baptiste Coudurier <baptiste dot coudurier at smartjog dot com>
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

/*
 * References
 * SMPTE 336M KLV Data Encoding Protocol Using Key-Length-Value
 * SMPTE 377M MXF File Format Specifications
 * SMPTE 378M Operational Pattern 1a
 * SMPTE 379M MXF Generic Container
 * SMPTE 381M Mapping MPEG Streams into the MXF Generic Container
 * SMPTE 382M Mapping AES3 and Broadcast Wave Audio into the MXF Generic Container
 * SMPTE 383M Mapping DV-DIF Data to the MXF Generic Container
 * SMPTE 2067-21 Interoperable Master Format — Application #2E
 *
 * Principle
 * Search for Track numbers which will identify essence element KLV packets.
 * Search for SourcePackage which define tracks which contains Track numbers.
 * Material Package contains tracks with reference to SourcePackage tracks.
 * Search for Descriptors (Picture, Sound) which contains codec info and parameters.
 * Assign Descriptors to correct Tracks.
 *
 * Metadata reading functions read Local Tags, get InstanceUID(0x3C0A) then add MetaDataSet to MXFContext.
 * Metadata parsing resolves Strong References to objects.
 *
 * Simple demuxer, only OP1A supported and some files might not work at all.
 * Only tracks with associated descriptors will be decoded. "Highly Desirable" SMPTE 377M D.1
 */

#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include <errno.h>

#include "libavutil/aes.h"
#include "libavutil/avstring.h"
#include "libavutil/mastering_display_metadata.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/bytestream.h"
#include "libavcodec/defs.h"
#include "libavcodec/h264_parse.h"
#include "libavcodec/internal.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/parseutils.h"
#include "libavutil/timecode.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/file_open.h"
#include "avformat.h"
#include "avlanguage.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "mxf.h"
#include "url.h"

#if HAVE_UNISTD_H
#include <unistd.h>
#endif
#if HAVE_FCNTL
#include <fcntl.h>
#endif

#define MXF_MAX_CHUNK_SIZE (32 << 20)
#define RUN_IN_MAX (65535+1)  // S377m-2004 section 5.5 and S377-1-2009 section 6.5, the +1 is to be slightly more tolerant

typedef enum {
    Header,
    BodyPartition,
    Footer
} MXFPartitionType;

typedef enum {
    OP1a = 1,
    OP1b,
    OP1c,
    OP2a,
    OP2b,
    OP2c,
    OP3a,
    OP3b,
    OP3c,
    OPAtom,
    OPSONYOpt,  /* FATE sample, violates the spec in places */
} MXFOP;

typedef enum {
    UnknownWrapped = 0,
    FrameWrapped,
    ClipWrapped,
} MXFWrappingScheme;

typedef struct MXFPartition {
    int closed;
    int complete;
    MXFPartitionType type;
    uint64_t previous_partition;
    int index_sid;
    int body_sid;
    int64_t essence_offset;         ///< absolute offset of essence
    int64_t essence_length;
    int32_t kag_size;
    int64_t header_byte_count;
    int64_t index_byte_count;
    int pack_length;
    int64_t pack_ofs;               ///< absolute offset of pack in file, including run-in
    int64_t pack_value_ofs;         ///< absolute offset of the first VALUE byte of the pack
    int64_t body_offset;
    KLVPacket first_essence_klv;
} MXFPartition;

typedef struct MXFMetadataSet {
    UID uid;
    uint64_t partition_score;
} MXFMetadataSet;

typedef struct MXFMetadataSetGroup {
    MXFMetadataSet **metadata_sets;
    int metadata_sets_count;
} MXFMetadataSetGroup;

typedef struct MXFCryptoContext {
    MXFMetadataSet meta;
    UID source_container_ul;
} MXFCryptoContext;

typedef struct MXFStructuralComponent {
    MXFMetadataSet meta;
    UID source_package_ul;
    UID source_package_uid;
    UID data_definition_ul;
    int64_t duration;
    int64_t start_position;
    int source_track_id;
} MXFStructuralComponent;

typedef struct MXFSequence {
    MXFMetadataSet meta;
    UID data_definition_ul;
    UID *structural_components_refs;
    int structural_components_count;
    int64_t duration;
} MXFSequence;

typedef struct MXFTimecodeComponent {
    MXFMetadataSet meta;
    int drop_frame;
    int start_frame;
    struct AVRational rate;
    AVTimecode tc;
} MXFTimecodeComponent;

typedef struct {
    MXFMetadataSet meta;
    UID input_segment_ref;
} MXFPulldownComponent;

typedef struct {
    MXFMetadataSet meta;
    UID *structural_components_refs;
    int structural_components_count;
    int64_t duration;
} MXFEssenceGroup;

typedef struct {
    MXFMetadataSet meta;
    char *name;
    char *value;
} MXFTaggedValue;

typedef struct {
    MXFMetadataSet meta;
    MXFSequence *sequence; /* mandatory, and only one */
    UID sequence_ref;
    int track_id;
    char *name;
    uint8_t track_number[4];
    AVRational edit_rate;
    int intra_only;
    uint64_t sample_count;
    int64_t original_duration; /* st->duration in SampleRate/EditRate units */
    int index_sid;
    int body_sid;
    MXFWrappingScheme wrapping;
    int edit_units_per_packet; /* how many edit units to read at a time (PCM, ClipWrapped) */
    int64_t origin;
} MXFTrack;

typedef struct MXFDescriptor {
    MXFMetadataSet meta;
    UID essence_container_ul;
    UID essence_codec_ul;
    UID codec_ul;
    AVRational sample_rate;
    AVRational aspect_ratio;
    int width;
    int height; /* Field height, not frame height */
    int frame_layout; /* See MXFFrameLayout enum */
    int video_line_map[2];
#define MXF_FIELD_DOMINANCE_DEFAULT 0
#define MXF_FIELD_DOMINANCE_FF 1 /* coded first, displayed first */
#define MXF_FIELD_DOMINANCE_FL 2 /* coded first, displayed last */
    int field_dominance;
    int channels;
    int bits_per_sample;
    int64_t duration; /* ContainerDuration optional property */
    unsigned int component_depth;
    unsigned int black_ref_level;
    unsigned int white_ref_level;
    unsigned int color_range;
    unsigned int horiz_subsampling;
    unsigned int vert_subsampling;
    UID *file_descriptors_refs;
    int file_descriptors_count;
    UID *sub_descriptors_refs;
    int sub_descriptors_count;
    int linked_track_id;
    uint8_t *extradata;
    int extradata_size;
    enum AVPixelFormat pix_fmt;
    UID color_primaries_ul;
    UID color_trc_ul;
    UID color_space_ul;
    AVMasteringDisplayMetadata *mastering;
    size_t mastering_size;
    AVContentLightMetadata *coll;
    size_t coll_size;
} MXFDescriptor;

typedef struct MXFMCASubDescriptor {
    MXFMetadataSet meta;
    UID uid;
    UID mca_link_id;
    UID soundfield_group_link_id;
    UID *group_of_soundfield_groups_link_id_refs;
    int group_of_soundfield_groups_link_id_count;
    UID mca_label_dictionary_id;
    int mca_channel_id;
    char *language;
} MXFMCASubDescriptor;

typedef struct MXFFFV1SubDescriptor {
    MXFMetadataSet meta;
    uint8_t *extradata;
    int extradata_size;
} MXFFFV1SubDescriptor;

typedef struct MXFAVCSubDescriptor {
    MXFMetadataSet meta;
    UID uid;
    int max_bit_rate;
} MXFAVCSubDescriptor;

typedef struct MXFIndexTableSegment {
    MXFMetadataSet meta;
    unsigned edit_unit_byte_count;
    int index_sid;
    int body_sid;
    AVRational index_edit_rate;
    uint64_t index_start_position;
    uint64_t index_duration;
    int8_t *temporal_offset_entries;
    int *flag_entries;
    uint64_t *stream_offset_entries;
    int nb_index_entries;
    int64_t offset;
} MXFIndexTableSegment;

typedef struct MXFPackage {
    MXFMetadataSet meta;
    UID package_uid;
    UID package_ul;
    UID *tracks_refs;
    int tracks_count;
    UID descriptor_ref;
    char *name;
    UID *comment_refs;
    int comment_count;
} MXFPackage;

typedef struct MXFEssenceContainerData {
    MXFMetadataSet meta;
    UID package_uid;
    UID package_ul;
    int index_sid;
    int body_sid;
} MXFEssenceContainerData;

typedef struct MXFGrowingIndex {
    int64_t  nb_entries;
    int64_t  entries_alloc;
    int64_t *offsets;
    int32_t *sizes;
    int8_t  *temporal_offsets;
    uint8_t *flags;
    /* running minimum of temporal_offsets[] seen so far by
     * mxf_growing_set_reordered_pts(); see that function's comment for why
     * this is a minimum here and not the negated maximum
     * mxf_compute_ptses_fake_index() uses. */
    int8_t   min_temporal_offset;
} MXFGrowingIndex;

/* Explicit role of this process with respect to a growing MXF's sidecar
 * index: at most one process ever holds WRITER (the fcntl() write lock on
 * growing_index_file); every other concurrent opener is a READER, which
 * trusts the sidecar the writer maintains and never independently measures
 * stride or indexes the reference track itself. */
enum MXFGrowingRole {
    MXF_GROWING_ROLE_NONE = 0,
    MXF_GROWING_ROLE_WRITER,
    MXF_GROWING_ROLE_READER,
};

/* decoded index table */
typedef struct MXFIndexTable {
    int index_sid;
    int body_sid;
    int nb_ptses;               /* number of PTSes or total duration of index */
    int64_t first_dts;          /* DTS = EditUnit + first_dts */
    int64_t *ptses;             /* maps EditUnit -> PTS */
    int nb_segments;
    MXFIndexTableSegment **segments;    /* sorted by IndexStartPosition */
    AVIndexEntry *fake_index;   /* used for calling ff_index_search_timestamp() */
    int8_t *offsets;            /* temporal offsets for display order to stored order conversion */
} MXFIndexTable;

typedef struct MXFContext {
    const AVClass *class;     /**< Class for private options. */
    MXFPartition *partitions;
    unsigned partitions_count;
    MXFOP op;
    UID *packages_refs;
    int packages_count;
    UID *essence_container_data_refs;
    int essence_container_data_count;
    MXFMetadataSetGroup metadata_set_groups[MetadataSetTypeNB];
    AVFormatContext *fc;
    struct AVAES *aesc;
    uint8_t *local_tags;
    int local_tags_count;
    uint64_t footer_partition;
    KLVPacket current_klv_data;
    int run_in;
    MXFPartition *current_partition;
    int parsing_backward;
    int64_t last_forward_tell;
    int last_forward_partition;
    int nb_index_tables;
    MXFIndexTable *index_tables;
    int eia608_extract;
    int skip_essence_parse;
    /* growing MXF support - see the block comment above
     * mxf_growing_select_ref_stream() */
    int         growing;                    ///< derived: growing_index_file set and no footer at open
    int         growing_poll_us;            ///< AVOption
    int64_t     growing_timeout_us;         ///< AVOption, 0 = wait forever
    int64_t     growing_index_stall_us;     ///< AVOption: reader's takeover-attempt threshold
    char       *growing_index_file;         ///< AVOption
    enum MXFGrowingRole growing_role;       ///< NONE / WRITER / READER
    int         growing_refused;            ///< sticky: mxf_growing_refuse_incompatible_essence()
    int         growing_ref_stream;         ///< reference stream index, -1 = none
    int         growing_clip_wrapped;       ///< reference track is ClipWrapped
    int64_t     growing_essence_offset;     ///< reference track's first essence element
    int64_t     growing_stride;             ///< bytes per reference edit unit, 0 = unknown
    int64_t     growing_elem_size;          ///< reference element size; == stride when clip-wrapped
    int         growing_stride_done;        ///< stride measurement has been attempted
    int         growing_stride_nb_obs;
    int64_t     growing_stride_obs[3];
    int64_t     growing_last_progress_us;       ///< writer: av_gettime_relative() at last essence progress
    int64_t     growing_last_index_progress_us; ///< reader: av_gettime_relative() at last sidecar advancement
    int64_t     growing_last_takeover_try_us;   ///< reader: throttles takeover attempts to once per growing_index_stall_us
    int64_t     growing_last_probe_us;      ///< av_gettime_relative() at last footer probe
    int64_t     growing_last_size;          ///< avio_size() at last observed growth
    int         growing_timed_out;          ///< sticky
    int         growing_file_closed;        ///< monotone 0 -> 1
    int64_t     growing_footer_offset;      ///< footer offset relative to run_in
    int64_t     growing_cur_duration;       ///< last published duration
    int64_t     growing_cp_start_offset;    ///< earliest KLV offset seen since the last content package boundary, -1 = none (OP1a offset fix)
    FILE       *growing_index_out;          ///< NULL = read-only or no sidecar
    int64_t     growing_index_entries_written;
    MXFGrowingIndex *growing_vbr_index;
    int64_t     growing_sidecar_duration;   ///< reader: last duration value read from the sidecar header
    int64_t     growing_index_resume_ofs;   ///< offset that (re)arms appending: first-time-writer start, or takeover/restart resume point
    int         growing_index_armed;
    int64_t     growing_index_last_ofs;     ///< offset of the last appended entry
    int         growing_index_disabled;     ///< sticky
    /* H.264/MPEG-2 elementary-stream reorder state (see mxf_growing_set_reordered_pts()).
     * growing_h264.poc is unused: the stock H.264 parser (growing_h264_parser
     * below) owns its own internal H264POCContext, so there is no hand-rolled
     * POC state machine to keep here - see mxf_growing_index_h264_temporal_offset(). */
    struct {
        H264POCContext poc;
        int64_t         gop_start_edit_unit;
        int             idr_poc;
        int             prev_poc;    ///< previous picture's raw output_picture_number, for poc_scale inference
        int             poc_scale;   ///< measured GCD of observed |POC deltas|, 0 = not yet known
        int             seen_idr;    ///< an IDR has been observed since open/resume; gop_start_edit_unit/idr_poc are valid
    } growing_h264;
    AVCodecParserContext *growing_h264_parser;  ///< av_parser_init(AV_CODEC_ID_H264), one per growing read (single reference track)
    AVCodecContext       *growing_h264_avctx;   ///< dummy context for growing_h264_parser, from the ref stream's codecpar
    int64_t     growing_mpeg2_gop_start_edit_unit;
    int         growing_mpeg2_seen_gop;  ///< a group_start_code has been observed since open/takeover/restart; gop_start_edit_unit is valid
} MXFContext;

/* NOTE: klv_offset is not set (-1) for local keys */
typedef int MXFMetadataReadFunc(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset);

typedef struct MXFMetadataReadTableEntry {
    const UID key;
    MXFMetadataReadFunc *read; /* if NULL then skip KLV */
    int ctx_size;
    enum MXFMetadataSetType type;
} MXFMetadataReadTableEntry;

/* partial keys to match */
static const uint8_t mxf_header_partition_pack_key[]       = { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x02 };
static const uint8_t mxf_essence_element_key[]             = { 0x06,0x0e,0x2b,0x34,0x01,0x02,0x01,0x01,0x0d,0x01,0x03,0x01 };
static const uint8_t mxf_avid_essence_element_key[]        = { 0x06,0x0e,0x2b,0x34,0x01,0x02,0x01,0x01,0x0e,0x04,0x03,0x01 };
static const uint8_t mxf_canopus_essence_element_key[]     = { 0x06,0x0e,0x2b,0x34,0x01,0x02,0x01,0x0a,0x0e,0x0f,0x03,0x01 };
static const uint8_t mxf_system_item_key_cp[]              = { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x03,0x01,0x04 };
static const uint8_t mxf_system_item_key_gc[]              = { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x03,0x01,0x14 };
static const uint8_t mxf_klv_key[]                         = { 0x06,0x0e,0x2b,0x34 };
static const uint8_t mxf_apple_coll_prefix[]               = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x0e,0x20,0x04,0x01,0x05,0x03,0x01 };

/* complete keys to match */
static const uint8_t mxf_crypto_source_container_ul[]      = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x09,0x06,0x01,0x01,0x02,0x02,0x00,0x00,0x00 };
static const uint8_t mxf_encrypted_triplet_key[]           = { 0x06,0x0e,0x2b,0x34,0x02,0x04,0x01,0x07,0x0d,0x01,0x03,0x01,0x02,0x7e,0x01,0x00 };
static const uint8_t mxf_encrypted_essence_container[]     = { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x07,0x0d,0x01,0x03,0x01,0x02,0x0b,0x01,0x00 };
static const uint8_t mxf_sony_mpeg4_extradata[]            = { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0e,0x06,0x06,0x02,0x02,0x01,0x00,0x00 };
static const uint8_t mxf_ffv1_extradata[]                  = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x04,0x01,0x06,0x0c,0x01,0x00,0x00,0x00 }; // FFV1InitializationMetadata
static const uint8_t mxf_avid_project_name[]               = { 0xa5,0xfb,0x7b,0x25,0xf6,0x15,0x94,0xb9,0x62,0xfc,0x37,0x17,0x49,0x2d,0x42,0xbf };
static const uint8_t mxf_jp2k_rsiz[]                       = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0a,0x04,0x01,0x06,0x03,0x01,0x00,0x00,0x00 };
static const uint8_t mxf_indirect_value_utf16le[]          = { 0x4c,0x00,0x02,0x10,0x01,0x00,0x00,0x00,0x00,0x06,0x0e,0x2b,0x34,0x01,0x04,0x01,0x01 };
static const uint8_t mxf_indirect_value_utf16be[]          = { 0x42,0x01,0x10,0x02,0x00,0x00,0x00,0x00,0x00,0x06,0x0e,0x2b,0x34,0x01,0x04,0x01,0x01 };
static const uint8_t mxf_apple_coll_max_cll[]              = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x0e,0x20,0x04,0x01,0x05,0x03,0x01,0x01 };
static const uint8_t mxf_apple_coll_max_fall[]             = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x0e,0x20,0x04,0x01,0x05,0x03,0x01,0x02 };

static const uint8_t mxf_mca_label_dictionary_id[]         = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x01,0x00,0x00,0x00 };
static const uint8_t mxf_mca_tag_symbol[]                  = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x02,0x00,0x00,0x00 };
static const uint8_t mxf_mca_tag_name[]                    = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x03,0x00,0x00,0x00 };
static const uint8_t mxf_group_of_soundfield_groups_link_id[] = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x04,0x00,0x00,0x00 };
static const uint8_t mxf_mca_link_id[]                     = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x05,0x00,0x00,0x00 };
static const uint8_t mxf_mca_channel_id[]                  = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x04,0x0a,0x00,0x00,0x00,0x00 };
static const uint8_t mxf_soundfield_group_link_id[]        = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0e,0x01,0x03,0x07,0x01,0x06,0x00,0x00,0x00 };
static const uint8_t mxf_mca_rfc5646_spoken_language[]     = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x0d,0x03,0x01,0x01,0x02,0x03,0x15,0x00,0x00 };

static const uint8_t mxf_sub_descriptor[]                  = { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0x09,0x06,0x01,0x01,0x04,0x06,0x10,0x00,0x00 };

static const uint8_t mxf_mastering_display_prefix[13]      = { FF_MXF_MasteringDisplay_PREFIX };
static const uint8_t mxf_mastering_display_uls[4][16] = {
    FF_MXF_MasteringDisplayPrimaries,
    FF_MXF_MasteringDisplayWhitePointChromaticity,
    FF_MXF_MasteringDisplayMaximumLuminance,
    FF_MXF_MasteringDisplayMinimumLuminance,
};

static const uint8_t mxf_avc_parameters_prefix[13] = { FF_MXF_AVCParameters_PREFIX };
static const uint8_t mxf_avc_parameters_uls[1][16] = {
        FF_MXF_AVCParametersMaximumBitrate,
};

#define IS_KLV_KEY(x, y) (!memcmp(x, y, sizeof(y)))

static void mxf_free_metadataset(MXFMetadataSet **ctx, enum MXFMetadataSetType type)
{
    MXFIndexTableSegment *seg;
    switch (type) {
    case Descriptor:
    case MultipleDescriptor:
        av_freep(&((MXFDescriptor *)*ctx)->extradata);
        av_freep(&((MXFDescriptor *)*ctx)->mastering);
        av_freep(&((MXFDescriptor *)*ctx)->coll);
        av_freep(&((MXFDescriptor *)*ctx)->file_descriptors_refs);
        av_freep(&((MXFDescriptor *)*ctx)->sub_descriptors_refs);
        break;
    case FFV1SubDescriptor:
        av_freep(&((MXFFFV1SubDescriptor *)*ctx)->extradata);
        break;
    case AudioChannelLabelSubDescriptor:
    case SoundfieldGroupLabelSubDescriptor:
    case GroupOfSoundfieldGroupsLabelSubDescriptor:
        av_freep(&((MXFMCASubDescriptor *)*ctx)->language);
        av_freep(&((MXFMCASubDescriptor *)*ctx)->group_of_soundfield_groups_link_id_refs);
        break;
    case Sequence:
        av_freep(&((MXFSequence *)*ctx)->structural_components_refs);
        break;
    case EssenceGroup:
        av_freep(&((MXFEssenceGroup *)*ctx)->structural_components_refs);
        break;
    case SourcePackage:
    case MaterialPackage:
        av_freep(&((MXFPackage *)*ctx)->tracks_refs);
        av_freep(&((MXFPackage *)*ctx)->name);
        av_freep(&((MXFPackage *)*ctx)->comment_refs);
        break;
    case TaggedValue:
        av_freep(&((MXFTaggedValue *)*ctx)->name);
        av_freep(&((MXFTaggedValue *)*ctx)->value);
        break;
    case Track:
        av_freep(&((MXFTrack *)*ctx)->name);
        break;
    case IndexTableSegment:
        seg = (MXFIndexTableSegment *)*ctx;
        av_freep(&seg->temporal_offset_entries);
        av_freep(&seg->flag_entries);
        av_freep(&seg->stream_offset_entries);
    default:
        break;
    }
    av_freep(ctx);
}

static int64_t klv_decode_ber_length(AVIOContext *pb, int *llen)
{
    uint64_t size = avio_r8(pb);
    if (size & 0x80) { /* long form */
        int bytes_num = size & 0x7f;
        /* SMPTE 379M 5.3.4 guarantee that bytes_num must not exceed 8 bytes */
        if (bytes_num > 8)
            return AVERROR_INVALIDDATA;
        if (llen)
            *llen = bytes_num + 1;
        size = 0;
        while (bytes_num--)
            size = size << 8 | avio_r8(pb);
    } else if (llen) {
        *llen = 1;
    }
    if (size > INT64_MAX)
        return AVERROR_INVALIDDATA;
    return size;
}

static int mxf_read_sync(AVIOContext *pb, const uint8_t *key, unsigned size)
{
    int i, b;
    for (i = 0; i < size && !avio_feof(pb); i++) {
        b = avio_r8(pb);
        if (b == key[0])
            i = 0;
        else if (b != key[i])
            i = -1;
    }
    return i == size;
}

// special case of mxf_read_sync for mxf_klv_key
static int mxf_read_sync_klv(AVIOContext *pb)
{
    uint32_t key = avio_rb32(pb);
    // key will never match mxf_klv_key on EOF
    if (key == AV_RB32(mxf_klv_key))
        return 1;

    while (!avio_feof(pb)) {
        key = (key << 8) | avio_r8(pb);
        if (key == AV_RB32(mxf_klv_key))
            return 1;
    }
    return 0;
}

static int klv_read_packet(MXFContext *mxf, KLVPacket *klv, AVIOContext *pb)
{
    int64_t length, pos;
    int llen;

    if (!mxf_read_sync_klv(pb))
        return AVERROR_INVALIDDATA;
    klv->offset = avio_tell(pb) - 4;
    if (klv->offset < mxf->run_in)
        return AVERROR_INVALIDDATA;

    memcpy(klv->key, mxf_klv_key, 4);
    int ret = ffio_read_size(pb, klv->key + 4, 12);
    if (ret < 0)
        return ret;
    length = klv_decode_ber_length(pb, &llen);
    if (length < 0)
        return length;
    klv->length = length;
    if (klv->offset > INT64_MAX - 16 - llen)
        return AVERROR_INVALIDDATA;

    pos = klv->offset + 16 + llen;
    if (pos > INT64_MAX - length)
        return AVERROR_INVALIDDATA;
    klv->next_klv = pos + length;
    return 0;
}

static int mxf_get_stream_index(AVFormatContext *s, KLVPacket *klv, int body_sid)
{
    for (int i = 0; i < s->nb_streams; i++) {
        MXFTrack *track = s->streams[i]->priv_data;
        /* SMPTE 379M 7.3 */
        if (track && (!body_sid || !track->body_sid || track->body_sid == body_sid) && !memcmp(klv->key + sizeof(mxf_essence_element_key), track->track_number, sizeof(track->track_number)))
            return i;
    }
    /* return 0 if only one stream, for OP Atom files with 0 as track number */
    return s->nb_streams == 1 && s->streams[0]->priv_data ? 0 : -1;
}

static int find_body_sid_by_absolute_offset(MXFContext *mxf, int64_t offset)
{
    // we look for partition where the offset is placed
    int a, b, m;
    int64_t pack_ofs;

    a = -1;
    b = mxf->partitions_count;

    while (b - a > 1) {
        m = (a + b) >> 1;
        pack_ofs = mxf->partitions[m].pack_ofs;
        if (pack_ofs <= offset)
            a = m;
        else
            b = m;
    }

    if (a == -1)
        return 0;
    return mxf->partitions[a].body_sid;
}

static int mxf_get_eia608_packet(AVFormatContext *s, AVStream *st, AVPacket *pkt, int64_t length)
{
    int count = avio_rb16(s->pb);
    int cdp_identifier, cdp_length, cdp_footer_id, ccdata_id, cc_count;
    int line_num, sample_coding, sample_count;
    int did, sdid, data_length;
    int ret;

    if (count > 1)
        av_log(s, AV_LOG_WARNING, "unsupported multiple ANC packets (%d) per KLV packet\n", count);

    for (int i = 0; i < count; i++) {
        if (length < 6) {
            av_log(s, AV_LOG_ERROR, "error reading s436m packet %"PRId64"\n", length);
            return AVERROR_INVALIDDATA;
        }
        line_num = avio_rb16(s->pb);
        avio_r8(s->pb); // wrapping type
        sample_coding = avio_r8(s->pb);
        sample_count = avio_rb16(s->pb);
        length -= 6 + 8 + sample_count;
        if (line_num != 9 && line_num != 11)
            continue;
        if (sample_coding == 7 || sample_coding == 8 || sample_coding == 9) {
            av_log(s, AV_LOG_WARNING, "unsupported s436m 10 bit sample coding\n");
            continue;
        }
        if (length < 0)
            return AVERROR_INVALIDDATA;

        avio_rb32(s->pb); // array count
        avio_rb32(s->pb); // array elem size
        did = avio_r8(s->pb);
        sdid = avio_r8(s->pb);
        data_length = avio_r8(s->pb);
        if (did != 0x61 || sdid != 1) {
            av_log(s, AV_LOG_WARNING, "unsupported did or sdid: %x %x\n", did, sdid);
            continue;
        }
        cdp_identifier = avio_rb16(s->pb); // cdp id
        if (cdp_identifier != 0x9669) {
            av_log(s, AV_LOG_ERROR, "wrong cdp identifier %x\n", cdp_identifier);
            return AVERROR_INVALIDDATA;
        }
        cdp_length = avio_r8(s->pb);
        avio_r8(s->pb); // cdp_frame_rate
        avio_r8(s->pb); // cdp_flags
        avio_rb16(s->pb); // cdp_hdr_sequence_cntr
        ccdata_id = avio_r8(s->pb); // ccdata_id
        if (ccdata_id != 0x72) {
            av_log(s, AV_LOG_ERROR, "wrong cdp data section %x\n", ccdata_id);
            return AVERROR_INVALIDDATA;
        }
        cc_count = avio_r8(s->pb) & 0x1f;
        ret = av_get_packet(s->pb, pkt, cc_count * 3);
        if (ret < 0)
            return ret;
        if (cdp_length - 9 - 4 <  cc_count * 3) {
            av_log(s, AV_LOG_ERROR, "wrong cdp size %d cc count %d\n", cdp_length, cc_count);
            return AVERROR_INVALIDDATA;
        }
        avio_skip(s->pb, data_length - 9 - 4 - cc_count * 3);
        cdp_footer_id = avio_r8(s->pb);
        if (cdp_footer_id != 0x74) {
            av_log(s, AV_LOG_ERROR, "wrong cdp footer section %x\n", cdp_footer_id);
            return AVERROR_INVALIDDATA;
        }
        avio_rb16(s->pb); // cdp_ftr_sequence_cntr
        avio_r8(s->pb); // packet_checksum
        break;
    }

    return 0;
}

/* XXX: use AVBitStreamFilter */
static int mxf_get_d10_aes3_packet(AVIOContext *pb, AVStream *st, AVPacket *pkt, int64_t length)
{
    const uint8_t *buf_ptr, *end_ptr;
    uint8_t *data_ptr;

    if (length > 61444) /* worst case PAL 1920 samples 8 channels */
        return AVERROR_INVALIDDATA;
    length = av_get_packet(pb, pkt, length);
    if (length < 0)
        return length;
    data_ptr = pkt->data;
    end_ptr = pkt->data + length;
    buf_ptr = pkt->data + 4; /* skip SMPTE 331M header */

    if (st->codecpar->ch_layout.nb_channels > 8)
        return AVERROR_INVALIDDATA;

    for (; end_ptr - buf_ptr >= st->codecpar->ch_layout.nb_channels * 4; ) {
        for (int i = 0; i < st->codecpar->ch_layout.nb_channels; i++) {
            uint32_t sample = bytestream_get_le32(&buf_ptr);
            if (st->codecpar->bits_per_coded_sample == 24)
                bytestream_put_le24(&data_ptr, (sample >> 4) & 0xffffff);
            else
                bytestream_put_le16(&data_ptr, (sample >> 12) & 0xffff);
        }
        // always 8 channels stored SMPTE 331M
        buf_ptr += 32 - st->codecpar->ch_layout.nb_channels * 4;
    }
    av_shrink_packet(pkt, data_ptr - pkt->data);
    return 0;
}

static int mxf_decrypt_triplet(AVFormatContext *s, AVPacket *pkt, KLVPacket *klv)
{
    static const uint8_t checkv[16] = {0x43, 0x48, 0x55, 0x4b, 0x43, 0x48, 0x55, 0x4b, 0x43, 0x48, 0x55, 0x4b, 0x43, 0x48, 0x55, 0x4b};
    MXFContext *mxf = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t end = avio_tell(pb) + klv->length;
    int64_t size;
    uint64_t orig_size;
    uint64_t plaintext_size;
    uint8_t ivec[16];
    uint8_t tmpbuf[16];
    int ret;
    int index;
    int body_sid;

    if (!mxf->aesc && s->key && s->keylen == 16) {
        mxf->aesc = av_aes_alloc();
        if (!mxf->aesc)
            return AVERROR(ENOMEM);
        av_aes_init(mxf->aesc, s->key, 128, 1);
    }
    // crypto context
    size = klv_decode_ber_length(pb, NULL);
    if (size < 0)
        return size;
    avio_skip(pb, size);
    // plaintext offset
    klv_decode_ber_length(pb ,NULL);
    plaintext_size = avio_rb64(pb);
    // source klv key
    klv_decode_ber_length(pb, NULL);
    avio_read(pb, klv->key, 16);
    if (!IS_KLV_KEY(klv, mxf_essence_element_key))
        return AVERROR_INVALIDDATA;

    body_sid = find_body_sid_by_absolute_offset(mxf, klv->offset);
    index = mxf_get_stream_index(s, klv, body_sid);
    if (index < 0)
        return AVERROR_INVALIDDATA;
    // source size
    klv_decode_ber_length(pb, NULL);
    orig_size = avio_rb64(pb);
    if (orig_size < plaintext_size)
        return AVERROR_INVALIDDATA;
    // enc. code
    size = klv_decode_ber_length(pb, NULL);
    if (size < 32 || size - 32 < orig_size || (int)orig_size != orig_size)
        return AVERROR_INVALIDDATA;
    avio_read(pb, ivec, 16);
    ret = ffio_read_size(pb, tmpbuf, 16);
    if (ret < 16)
        return ret;
    if (mxf->aesc)
        av_aes_crypt(mxf->aesc, tmpbuf, tmpbuf, 1, ivec, 1);
    if (memcmp(tmpbuf, checkv, 16))
        av_log(s, AV_LOG_ERROR, "probably incorrect decryption key\n");
    size -= 32;
    size = av_get_packet(pb, pkt, size);
    if (size < 0)
        return size;
    else if (size < plaintext_size)
        return AVERROR_INVALIDDATA;
    size -= plaintext_size;
    if (mxf->aesc)
        av_aes_crypt(mxf->aesc, &pkt->data[plaintext_size],
                     &pkt->data[plaintext_size], size >> 4, ivec, 1);
    av_shrink_packet(pkt, orig_size);
    pkt->stream_index = index;
    avio_skip(pb, end - avio_tell(pb));
    return 0;
}

static int mxf_read_primer_pack(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFContext *mxf = arg;
    int item_num = avio_rb32(pb);
    int item_len = avio_rb32(pb);

    if (item_len != 18) {
        avpriv_request_sample(pb, "Primer pack item length %d", item_len);
        return AVERROR_PATCHWELCOME;
    }
    if (item_num > 65536 || item_num < 0) {
        av_log(mxf->fc, AV_LOG_ERROR, "item_num %d is too large\n", item_num);
        return AVERROR_INVALIDDATA;
    }
    if (mxf->local_tags)
        av_log(mxf->fc, AV_LOG_VERBOSE, "Multiple primer packs\n");
    av_free(mxf->local_tags);
    mxf->local_tags_count = 0;
    mxf->local_tags = av_calloc(item_num, item_len);
    if (!mxf->local_tags)
        return AVERROR(ENOMEM);
    mxf->local_tags_count = item_num;
    avio_read(pb, mxf->local_tags, item_num*item_len);
    return 0;
}

static int mxf_read_partition_pack(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFContext *mxf = arg;
    AVFormatContext *s = mxf->fc;
    MXFPartition *partition, *tmp_part;
    UID op;
    uint64_t footer_partition;
    uint32_t nb_essence_containers;
    uint64_t this_partition;
    int ret;

    if (mxf->partitions_count >= INT_MAX / 2)
        return AVERROR_INVALIDDATA;

    av_assert0(klv_offset >= mxf->run_in);

    tmp_part = av_realloc_array(mxf->partitions, mxf->partitions_count + 1, sizeof(*mxf->partitions));
    if (!tmp_part)
        return AVERROR(ENOMEM);
    mxf->partitions = tmp_part;

    if (mxf->parsing_backward) {
        /* insert the new partition pack in the middle
         * this makes the entries in mxf->partitions sorted by offset */
        memmove(&mxf->partitions[mxf->last_forward_partition+1],
                &mxf->partitions[mxf->last_forward_partition],
                (mxf->partitions_count - mxf->last_forward_partition)*sizeof(*mxf->partitions));
        partition = mxf->current_partition = &mxf->partitions[mxf->last_forward_partition];
    } else {
        mxf->last_forward_partition++;
        partition = mxf->current_partition = &mxf->partitions[mxf->partitions_count];
    }

    memset(partition, 0, sizeof(*partition));
    mxf->partitions_count++;
    partition->pack_length = avio_tell(pb) - klv_offset + size;
    partition->pack_value_ofs = avio_tell(pb);
    partition->pack_ofs    = klv_offset;

    switch(uid[13]) {
    case 2:
        partition->type = Header;
        break;
    case 3:
        partition->type = BodyPartition;
        break;
    case 4:
        partition->type = Footer;
        break;
    default:
        av_log(mxf->fc, AV_LOG_ERROR, "unknown partition type %i\n", uid[13]);
        return AVERROR_INVALIDDATA;
    }

    /* consider both footers to be closed (there is only Footer and CompleteFooter) */
    partition->closed = partition->type == Footer || !(uid[14] & 1);
    partition->complete = uid[14] > 2;
    avio_skip(pb, 4);
    partition->kag_size = avio_rb32(pb);
    this_partition = avio_rb64(pb);
    if (this_partition != klv_offset - mxf->run_in) {
        av_log(mxf->fc, AV_LOG_ERROR,
               "this_partition %"PRId64" mismatches %"PRId64"\n",
               this_partition, klv_offset - mxf->run_in);
        return AVERROR_INVALIDDATA;
    }
    partition->previous_partition = avio_rb64(pb);
    footer_partition = avio_rb64(pb);
    partition->header_byte_count = avio_rb64(pb);
    partition->index_byte_count = avio_rb64(pb);
    partition->index_sid = avio_rb32(pb);
    partition->body_offset = avio_rb64(pb);
    partition->body_sid = avio_rb32(pb);
    if (partition->body_offset < 0)
        return AVERROR_INVALIDDATA;

    ret = ffio_read_size(pb, op, sizeof(UID));
    if (ret < 0) {
        av_log(mxf->fc, AV_LOG_ERROR, "Failed reading UID\n");
        return ret;
    }
    nb_essence_containers = avio_rb32(pb);

    if (partition->type == Header) {
        char str[36];
        snprintf(str, sizeof(str), "%08x.%08x.%08x.%08x", AV_RB32(&op[0]), AV_RB32(&op[4]), AV_RB32(&op[8]), AV_RB32(&op[12]));
        av_dict_set(&s->metadata, "operational_pattern_ul", str, 0);
    }

    if (this_partition &&
        partition->previous_partition == this_partition) {
        av_log(mxf->fc, AV_LOG_ERROR,
               "PreviousPartition equal to ThisPartition %"PRIx64"\n",
               partition->previous_partition);
        /* override with the actual previous partition offset */
        if (!mxf->parsing_backward && mxf->last_forward_partition > 1) {
            MXFPartition *prev =
                mxf->partitions + mxf->last_forward_partition - 2;
            partition->previous_partition = prev->pack_ofs - mxf->run_in;
        }
        /* if no previous body partition are found point to the header
         * partition */
        if (partition->previous_partition == this_partition)
            partition->previous_partition = 0;
        av_log(mxf->fc, AV_LOG_ERROR,
               "Overriding PreviousPartition with %"PRIx64"\n",
               partition->previous_partition);
    }

    /* some files don't have FooterPartition set in every partition */
    if (footer_partition) {
        if (mxf->footer_partition && mxf->footer_partition != footer_partition) {
            av_log(mxf->fc, AV_LOG_ERROR,
                   "inconsistent FooterPartition value: %"PRIu64" != %"PRIu64"\n",
                   mxf->footer_partition, footer_partition);
        } else {
            mxf->footer_partition = footer_partition;
        }
    }

    av_log(mxf->fc, AV_LOG_TRACE,
            "PartitionPack: ThisPartition = 0x%"PRIX64
            ", PreviousPartition = 0x%"PRIX64", "
            "FooterPartition = 0x%"PRIX64", IndexSID = %i, BodySID = %i\n",
            this_partition,
            partition->previous_partition, footer_partition,
            partition->index_sid, partition->body_sid);

    /* sanity check PreviousPartition if set */
    //NOTE: this isn't actually enough, see mxf_seek_to_previous_partition()
    if (partition->previous_partition &&
        mxf->run_in + partition->previous_partition >= klv_offset) {
        av_log(mxf->fc, AV_LOG_ERROR,
               "PreviousPartition points to this partition or forward\n");
        return AVERROR_INVALIDDATA;
    }

    if      (op[12] == 1 && op[13] == 1) mxf->op = OP1a;
    else if (op[12] == 1 && op[13] == 2) mxf->op = OP1b;
    else if (op[12] == 1 && op[13] == 3) mxf->op = OP1c;
    else if (op[12] == 2 && op[13] == 1) mxf->op = OP2a;
    else if (op[12] == 2 && op[13] == 2) mxf->op = OP2b;
    else if (op[12] == 2 && op[13] == 3) mxf->op = OP2c;
    else if (op[12] == 3 && op[13] == 1) mxf->op = OP3a;
    else if (op[12] == 3 && op[13] == 2) mxf->op = OP3b;
    else if (op[12] == 3 && op[13] == 3) mxf->op = OP3c;
    else if (op[12] == 64&& op[13] == 1) mxf->op = OPSONYOpt;
    else if (op[12] == 0x10) {
        /* SMPTE 390m: "There shall be exactly one essence container"
         * The following block deals with files that violate this, namely:
         * 2011_DCPTEST_24FPS.V.mxf - two ECs, OP1a
         * abcdefghiv016f56415e.mxf - zero ECs, OPAtom, output by Avid AirSpeed */
        if (nb_essence_containers != 1) {
            MXFOP mxfop = nb_essence_containers ? OP1a : OPAtom;

            /* only nag once */
            if (!mxf->op)
                av_log(mxf->fc, AV_LOG_WARNING,
                       "\"OPAtom\" with %"PRIu32" ECs - assuming %s\n",
                       nb_essence_containers,
                       mxfop == OP1a ? "OP1a" : "OPAtom");

            mxf->op = mxfop;
        } else
            mxf->op = OPAtom;
    } else {
        av_log(mxf->fc, AV_LOG_ERROR, "unknown operational pattern: %02xh %02xh - guessing OP1a\n", op[12], op[13]);
        mxf->op = OP1a;
    }

    if (partition->kag_size <= 0 || partition->kag_size > (1 << 20)) {
        av_log(mxf->fc, AV_LOG_WARNING, "invalid KAGSize %"PRId32" - guessing ",
               partition->kag_size);

        if (mxf->op == OPSONYOpt)
            partition->kag_size = 512;
        else
            partition->kag_size = 1;

        av_log(mxf->fc, AV_LOG_WARNING, "%"PRId32"\n", partition->kag_size);
    }

    return 0;
}

static uint64_t partition_score(MXFPartition *p)
{
    uint64_t score;
    if (!p)
        return 0;
    if (p->type == Footer)
        score = 5;
    else if (p->complete)
        score = 4;
    else if (p->closed)
        score = 3;
    else
        score = 1;
    return (score << 60) | ((uint64_t)p->pack_ofs >> 4);
}

static int mxf_add_metadata_set(MXFContext *mxf, MXFMetadataSet **metadata_set, enum MXFMetadataSetType type)
{
    MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[type];
    int ret;

    // Index Table is special because it might be added manually without
    // partition and we iterate through all instances of them. Also some files
    // use the same Instance UID for different index tables...
    if (type != IndexTableSegment) {
        for (int i = 0; i < mg->metadata_sets_count; i++) {
            if (!memcmp((*metadata_set)->uid, mg->metadata_sets[i]->uid, 16)) {
                uint64_t old_s = mg->metadata_sets[i]->partition_score;
                uint64_t new_s = (*metadata_set)->partition_score;
                if (old_s > new_s) {
                     mxf_free_metadataset(metadata_set, type);
                     return 0;
                }
            }
        }
    }

    ret = av_dynarray_add_nofree(&mg->metadata_sets, &mg->metadata_sets_count, *metadata_set);
    if (ret < 0) {
        mxf_free_metadataset(metadata_set, type);
        return ret;
    }
    return 0;
}

static int mxf_read_cryptographic_context(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFCryptoContext *cryptocontext = arg;
    if (size != 16)
        return AVERROR_INVALIDDATA;
    if (IS_KLV_KEY(uid, mxf_crypto_source_container_ul))
        avio_read(pb, cryptocontext->source_container_ul, 16);
    return 0;
}

static int mxf_read_strong_ref_array(AVIOContext *pb, UID **refs, int *count)
{
    int64_t ret;
    unsigned c = avio_rb32(pb);

    //avio_read() used int
    if (c > INT_MAX / sizeof(UID))
        return AVERROR_PATCHWELCOME;
    *count = c;

    av_free(*refs);
    *refs = av_malloc_array(*count, sizeof(UID));
    if (!*refs) {
        *count = 0;
        return AVERROR(ENOMEM);
    }
    avio_skip(pb, 4); /* useless size of objects, always 16 according to specs */
    ret = avio_read(pb, (uint8_t *)*refs, *count * sizeof(UID));
    if (ret != *count * sizeof(UID)) {
        *count = ret < 0 ? 0   : ret / sizeof(UID);
        return   ret < 0 ? ret : AVERROR_INVALIDDATA;
    }

    return 0;
}

static inline int mxf_read_us_ascii_string(AVIOContext *pb, int size, char** str)
{
    int ret;
    size_t buf_size;

    if (size < 0 || size > INT_MAX - 1)
        return AVERROR(EINVAL);

    buf_size = size + 1;
    av_free(*str);
    *str = av_malloc(buf_size);
    if (!*str)
        return AVERROR(ENOMEM);

    ret = avio_get_str(pb, size, *str, buf_size);

    if (ret < 0) {
        av_freep(str);
        return ret;
    }

    return ret;
}

static inline int mxf_read_utf16_string(AVIOContext *pb, int size, char** str, int be)
{
    int ret;
    size_t buf_size;

    if (size < 0 || size > INT_MAX/2)
        return AVERROR(EINVAL);

    buf_size = size + size / 2 + 1;
    av_free(*str);
    *str = av_malloc(buf_size);
    if (!*str)
        return AVERROR(ENOMEM);

    if (be)
        ret = avio_get_str16be(pb, size, *str, buf_size);
    else
        ret = avio_get_str16le(pb, size, *str, buf_size);

    if (ret < 0) {
        av_freep(str);
        return ret;
    }

    return ret;
}

#define READ_STR16(type, big_endian)                                               \
static int mxf_read_utf16 ## type ##_string(AVIOContext *pb, int size, char** str) \
{                                                                                  \
return mxf_read_utf16_string(pb, size, str, big_endian);                           \
}
READ_STR16(be, 1)
READ_STR16(le, 0)
#undef READ_STR16

static int mxf_read_content_storage(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFContext *mxf = arg;
    switch (tag) {
    case 0x1901:
        if (mxf->packages_refs)
            av_log(mxf->fc, AV_LOG_VERBOSE, "Multiple packages_refs\n");
        return mxf_read_strong_ref_array(pb, &mxf->packages_refs, &mxf->packages_count);
    case 0x1902:
        return mxf_read_strong_ref_array(pb, &mxf->essence_container_data_refs, &mxf->essence_container_data_count);
    }
    return 0;
}

static int mxf_read_source_clip(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFStructuralComponent *source_clip = arg;
    switch(tag) {
    case 0x0202:
        source_clip->duration = avio_rb64(pb);
        break;
    case 0x1201:
        source_clip->start_position = avio_rb64(pb);
        break;
    case 0x1101:
        /* UMID, only get last 16 bytes */
        avio_read(pb, source_clip->source_package_ul, 16);
        avio_read(pb, source_clip->source_package_uid, 16);
        break;
    case 0x1102:
        source_clip->source_track_id = avio_rb32(pb);
        break;
    }
    return 0;
}

static int mxf_read_timecode_component(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFTimecodeComponent *mxf_timecode = arg;
    switch(tag) {
    case 0x1501:
        mxf_timecode->start_frame = avio_rb64(pb);
        break;
    case 0x1502:
        mxf_timecode->rate = (AVRational){avio_rb16(pb), 1};
        break;
    case 0x1503:
        mxf_timecode->drop_frame = avio_r8(pb);
        break;
    }
    return 0;
}

static int mxf_read_avc_sub_descriptor(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFAVCSubDescriptor *mxf_avc_sub_descriptor = arg;
    if (IS_KLV_KEY(uid, mxf_avc_parameters_prefix)) {
        if (IS_KLV_KEY(uid, mxf_avc_parameters_uls[0])) {
            mxf_avc_sub_descriptor->max_bit_rate = avio_rb32(pb);
        }
    }
    return 0;
}

static int mxf_read_pulldown_component(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFPulldownComponent *mxf_pulldown = arg;
    switch(tag) {
    case 0x0d01:
        avio_read(pb, mxf_pulldown->input_segment_ref, 16);
        break;
    }
    return 0;
}

static int mxf_read_track(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFTrack *track = arg;
    switch(tag) {
    case 0x4801:
        track->track_id = avio_rb32(pb);
        break;
    case 0x4804:
        avio_read(pb, track->track_number, 4);
        break;
    case 0x4802:
        mxf_read_utf16be_string(pb, size, &track->name);
        break;
    case 0x4b01:
        track->edit_rate.num = avio_rb32(pb);
        track->edit_rate.den = avio_rb32(pb);
        break;
    case 0x4b02:
        track->origin = avio_rb64(pb);
        break;
    case 0x4803:
        avio_read(pb, track->sequence_ref, 16);
        break;
    }
    return 0;
}

static int mxf_read_sequence(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFSequence *sequence = arg;
    switch(tag) {
    case 0x0202:
        sequence->duration = avio_rb64(pb);
        break;
    case 0x0201:
        avio_read(pb, sequence->data_definition_ul, 16);
        break;
    case 0x1001:
        return mxf_read_strong_ref_array(pb, &sequence->structural_components_refs,
                                             &sequence->structural_components_count);
    }
    return 0;
}

static int mxf_read_essence_group(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFEssenceGroup *essence_group = arg;
    switch (tag) {
    case 0x0202:
        essence_group->duration = avio_rb64(pb);
        break;
    case 0x0501:
        return mxf_read_strong_ref_array(pb, &essence_group->structural_components_refs,
                                             &essence_group->structural_components_count);
    }
    return 0;
}

static int mxf_read_package(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFPackage *package = arg;
    switch(tag) {
    case 0x4403:
        return mxf_read_strong_ref_array(pb, &package->tracks_refs,
                                             &package->tracks_count);
    case 0x4401:
        /* UMID */
        avio_read(pb, package->package_ul, 16);
        avio_read(pb, package->package_uid, 16);
        break;
    case 0x4701:
        avio_read(pb, package->descriptor_ref, 16);
        break;
    case 0x4402:
        return mxf_read_utf16be_string(pb, size, &package->name);
    case 0x4406:
        return mxf_read_strong_ref_array(pb, &package->comment_refs,
                                             &package->comment_count);
    }
    return 0;
}

static int mxf_read_essence_container_data(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFEssenceContainerData *essence_data = arg;
    switch(tag) {
        case 0x2701:
            /* linked package umid UMID */
            avio_read(pb, essence_data->package_ul, 16);
            avio_read(pb, essence_data->package_uid, 16);
            break;
        case 0x3f06:
            essence_data->index_sid = avio_rb32(pb);
            break;
        case 0x3f07:
            essence_data->body_sid = avio_rb32(pb);
            break;
    }
    return 0;
}

static int mxf_read_index_entry_array(AVIOContext *pb, MXFIndexTableSegment *segment)
{
    int i, length;
    uint32_t nb_index_entries;

    if (segment->temporal_offset_entries)
        return AVERROR_INVALIDDATA;

    nb_index_entries = avio_rb32(pb);
    if (nb_index_entries > INT_MAX)
        return AVERROR_INVALIDDATA;
    segment->nb_index_entries = nb_index_entries;

    length = avio_rb32(pb);
    if(segment->nb_index_entries && length < 11)
        return AVERROR_INVALIDDATA;

    if (!FF_ALLOC_TYPED_ARRAY(segment->temporal_offset_entries, segment->nb_index_entries) ||
        !FF_ALLOC_TYPED_ARRAY(segment->flag_entries           , segment->nb_index_entries) ||
        !FF_ALLOC_TYPED_ARRAY(segment->stream_offset_entries  , segment->nb_index_entries)) {
        av_freep(&segment->temporal_offset_entries);
        av_freep(&segment->flag_entries);
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < segment->nb_index_entries; i++) {
        if(avio_feof(pb))
            return AVERROR_INVALIDDATA;
        segment->temporal_offset_entries[i] = avio_r8(pb);
        avio_r8(pb);                                        /* KeyFrameOffset */
        segment->flag_entries[i] = avio_r8(pb);
        segment->stream_offset_entries[i] = avio_rb64(pb);
        avio_skip(pb, length - 11);
    }
    return 0;
}

static int mxf_read_index_table_segment(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFIndexTableSegment *segment = arg;
    switch(tag) {
    case 0x3F05:
        segment->edit_unit_byte_count = avio_rb32(pb);
        av_log(NULL, AV_LOG_TRACE, "EditUnitByteCount %d\n", segment->edit_unit_byte_count);
        break;
    case 0x3F06:
        segment->index_sid = avio_rb32(pb);
        av_log(NULL, AV_LOG_TRACE, "IndexSID %d\n", segment->index_sid);
        break;
    case 0x3F07:
        segment->body_sid = avio_rb32(pb);
        av_log(NULL, AV_LOG_TRACE, "BodySID %d\n", segment->body_sid);
        break;
    case 0x3F0A:
        av_log(NULL, AV_LOG_TRACE, "IndexEntryArray found\n");
        return mxf_read_index_entry_array(pb, segment);
    case 0x3F0B:
        segment->index_edit_rate.num = avio_rb32(pb);
        segment->index_edit_rate.den = avio_rb32(pb);
        if ((segment->index_edit_rate.num == 0 && segment->index_edit_rate.den == 0)) {
            segment->index_edit_rate.den = 1;
        }
        else if (segment->index_edit_rate.num <= 0 ||
            segment->index_edit_rate.den <= 0)
            return AVERROR_INVALIDDATA;
        av_log(NULL, AV_LOG_TRACE, "IndexEditRate %d/%d\n", segment->index_edit_rate.num,
                segment->index_edit_rate.den);
        break;
    case 0x3F0C:
        segment->index_start_position = avio_rb64(pb);
        av_log(NULL, AV_LOG_TRACE, "IndexStartPosition %"PRId64"\n", segment->index_start_position);
        break;
    case 0x3F0D:
        segment->index_duration = avio_rb64(pb);
        av_log(NULL, AV_LOG_TRACE, "IndexDuration %"PRId64"\n", segment->index_duration);
        break;
    }
    return 0;
}

static void mxf_read_pixel_layout(AVIOContext *pb, MXFDescriptor *descriptor)
{
    int code, value, ofs = 0;
    char layout[16] = {0}; /* not for printing, may end up not terminated on purpose */

    do {
        code = avio_r8(pb);
        value = avio_r8(pb);
        av_log(NULL, AV_LOG_TRACE, "pixel layout: code %#x\n", code);

        if (ofs <= 14) {
            layout[ofs++] = code;
            layout[ofs++] = value;
        } else
            break;  /* don't read byte by byte on sneaky files filled with lots of non-zeroes */
    } while (code != 0); /* SMPTE 377M E.2.46 */

    ff_mxf_decode_pixel_layout(layout, &descriptor->pix_fmt);
}

static int mxf_read_generic_descriptor(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFDescriptor *descriptor = arg;
    int entry_count, entry_size;

    switch(tag) {
    case 0x3F01:
        return mxf_read_strong_ref_array(pb, &descriptor->file_descriptors_refs,
                                             &descriptor->file_descriptors_count);
    case 0x3002: /* ContainerDuration */
        descriptor->duration = avio_rb64(pb);
        break;
    case 0x3004:
        avio_read(pb, descriptor->essence_container_ul, 16);
        break;
    case 0x3005:
        avio_read(pb, descriptor->codec_ul, 16);
        break;
    case 0x3006:
        descriptor->linked_track_id = avio_rb32(pb);
        break;
    case 0x3201: /* PictureEssenceCoding */
        avio_read(pb, descriptor->essence_codec_ul, 16);
        break;
    case 0x3203:
        descriptor->width = avio_rb32(pb);
        break;
    case 0x3202:
        descriptor->height = avio_rb32(pb);
        break;
    case 0x320C:
        descriptor->frame_layout = avio_r8(pb);
        break;
    case 0x320D:
        entry_count = avio_rb32(pb);
        entry_size = avio_rb32(pb);
        if (entry_size == 4) {
            if (entry_count > 0)
                descriptor->video_line_map[0] = avio_rb32(pb);
            else
                descriptor->video_line_map[0] = 0;
            if (entry_count > 1)
                descriptor->video_line_map[1] = avio_rb32(pb);
            else
                descriptor->video_line_map[1] = 0;
        } else
            av_log(NULL, AV_LOG_WARNING, "VideoLineMap element size %d currently not supported\n", entry_size);
        break;
    case 0x320E:
        descriptor->aspect_ratio.num = avio_rb32(pb);
        descriptor->aspect_ratio.den = avio_rb32(pb);
        break;
    case 0x3210:
        avio_read(pb, descriptor->color_trc_ul, 16);
        break;
    case 0x3212:
        descriptor->field_dominance = avio_r8(pb);
        break;
    case 0x3219:
        avio_read(pb, descriptor->color_primaries_ul, 16);
        break;
    case 0x321A:
        avio_read(pb, descriptor->color_space_ul, 16);
        break;
    case 0x3301:
        descriptor->component_depth = avio_rb32(pb);
        break;
    case 0x3302:
        descriptor->horiz_subsampling = avio_rb32(pb);
        break;
    case 0x3304:
        descriptor->black_ref_level = avio_rb32(pb);
        break;
    case 0x3305:
        descriptor->white_ref_level = avio_rb32(pb);
        break;
    case 0x3306:
        descriptor->color_range = avio_rb32(pb);
        break;
    case 0x3308:
        descriptor->vert_subsampling = avio_rb32(pb);
        break;
    case 0x3D03:
        descriptor->sample_rate.num = avio_rb32(pb);
        descriptor->sample_rate.den = avio_rb32(pb);
        break;
    case 0x3D06: /* SoundEssenceCompression */
        avio_read(pb, descriptor->essence_codec_ul, 16);
        break;
    case 0x3D07:
        descriptor->channels = avio_rb32(pb);
        break;
    case 0x3D01:
        descriptor->bits_per_sample = avio_rb32(pb);
        break;
    case 0x3401:
        mxf_read_pixel_layout(pb, descriptor);
        break;
    default:
        /* Private uid used by SONY C0023S01.mxf */
        if (IS_KLV_KEY(uid, mxf_sony_mpeg4_extradata)) {
            if (descriptor->extradata)
                av_log(NULL, AV_LOG_WARNING, "Duplicate sony_mpeg4_extradata\n");
            av_free(descriptor->extradata);
            descriptor->extradata_size = 0;
            descriptor->extradata = av_mallocz(size + AV_INPUT_BUFFER_PADDING_SIZE);
            if (!descriptor->extradata)
                return AVERROR(ENOMEM);
            descriptor->extradata_size = size;
            avio_read(pb, descriptor->extradata, size);
        }
        if (IS_KLV_KEY(uid, mxf_jp2k_rsiz)) {
            uint32_t rsiz = avio_rb16(pb);
            if (rsiz == AV_PROFILE_JPEG2000_DCINEMA_2K ||
                rsiz == AV_PROFILE_JPEG2000_DCINEMA_4K)
                descriptor->pix_fmt = AV_PIX_FMT_XYZ12;
        }
        if (IS_KLV_KEY(uid, mxf_mastering_display_prefix)) {
            if (!descriptor->mastering) {
                descriptor->mastering = av_mastering_display_metadata_alloc_size(&descriptor->mastering_size);
                if (!descriptor->mastering)
                    return AVERROR(ENOMEM);
            }
            if (IS_KLV_KEY(uid, mxf_mastering_display_uls[0])) {
                for (int i = 0; i < 3; i++) {
                    /* Order: large x, large y, other (i.e. RGB) */
                    descriptor->mastering->display_primaries[i][0] = av_make_q(avio_rb16(pb), FF_MXF_MASTERING_CHROMA_DEN);
                    descriptor->mastering->display_primaries[i][1] = av_make_q(avio_rb16(pb), FF_MXF_MASTERING_CHROMA_DEN);
                }
                /* Check we have seen mxf_mastering_display_white_point_chromaticity */
                if (descriptor->mastering->white_point[0].den != 0)
                    descriptor->mastering->has_primaries = 1;
            }
            if (IS_KLV_KEY(uid, mxf_mastering_display_uls[1])) {
                descriptor->mastering->white_point[0] = av_make_q(avio_rb16(pb), FF_MXF_MASTERING_CHROMA_DEN);
                descriptor->mastering->white_point[1] = av_make_q(avio_rb16(pb), FF_MXF_MASTERING_CHROMA_DEN);
                /* Check we have seen mxf_mastering_display_primaries */
                if (descriptor->mastering->display_primaries[0][0].den != 0)
                    descriptor->mastering->has_primaries = 1;
            }
            if (IS_KLV_KEY(uid, mxf_mastering_display_uls[2])) {
                descriptor->mastering->max_luminance = av_make_q(avio_rb32(pb), FF_MXF_MASTERING_LUMA_DEN);
                /* Check we have seen mxf_mastering_display_minimum_luminance */
                if (descriptor->mastering->min_luminance.den != 0)
                    descriptor->mastering->has_luminance = 1;
            }
            if (IS_KLV_KEY(uid, mxf_mastering_display_uls[3])) {
                descriptor->mastering->min_luminance = av_make_q(avio_rb32(pb), FF_MXF_MASTERING_LUMA_DEN);
                /* Check we have seen mxf_mastering_display_maximum_luminance */
                if (descriptor->mastering->max_luminance.den != 0)
                    descriptor->mastering->has_luminance = 1;
            }
        }
        if (IS_KLV_KEY(uid, mxf_apple_coll_prefix)) {
            if (!descriptor->coll) {
                descriptor->coll = av_content_light_metadata_alloc(&descriptor->coll_size);
                if (!descriptor->coll)
                    return AVERROR(ENOMEM);
            }
            if (IS_KLV_KEY(uid, mxf_apple_coll_max_cll)) {
                descriptor->coll->MaxCLL = avio_rb16(pb);
            }
            if (IS_KLV_KEY(uid, mxf_apple_coll_max_fall)) {
                descriptor->coll->MaxFALL = avio_rb16(pb);
            }
        }

        if (IS_KLV_KEY(uid, mxf_sub_descriptor))
            return mxf_read_strong_ref_array(pb, &descriptor->sub_descriptors_refs, &descriptor->sub_descriptors_count);

        break;
    }
    return 0;
}

static int mxf_read_mca_sub_descriptor(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFMCASubDescriptor *mca_sub_descriptor = arg;

    if (IS_KLV_KEY(uid, mxf_mca_label_dictionary_id))
        avio_read(pb, mca_sub_descriptor->mca_label_dictionary_id, 16);

    if (IS_KLV_KEY(uid, mxf_mca_link_id))
        avio_read(pb, mca_sub_descriptor->mca_link_id, 16);

    if (IS_KLV_KEY(uid, mxf_soundfield_group_link_id))
        avio_read(pb, mca_sub_descriptor->soundfield_group_link_id, 16);

    if (IS_KLV_KEY(uid, mxf_group_of_soundfield_groups_link_id))
        return mxf_read_strong_ref_array(pb, &mca_sub_descriptor->group_of_soundfield_groups_link_id_refs, &mca_sub_descriptor->group_of_soundfield_groups_link_id_count);

    if (IS_KLV_KEY(uid, mxf_mca_channel_id))
        mca_sub_descriptor->mca_channel_id = avio_rb32(pb);

    if (IS_KLV_KEY(uid, mxf_mca_rfc5646_spoken_language))
        return mxf_read_us_ascii_string(pb, size, &mca_sub_descriptor->language);

    return 0;
}

static int mxf_read_ffv1_sub_descriptor(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFFFV1SubDescriptor *ffv1_sub_descriptor = arg;

    if (IS_KLV_KEY(uid, mxf_ffv1_extradata) && size <= INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE ) {
        if (ffv1_sub_descriptor->extradata)
            av_log(NULL, AV_LOG_WARNING, "Duplicate ffv1_extradata\n");
        av_free(ffv1_sub_descriptor->extradata);
        ffv1_sub_descriptor->extradata_size = 0;
        ffv1_sub_descriptor->extradata = av_mallocz(size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!ffv1_sub_descriptor->extradata)
            return AVERROR(ENOMEM);
        ffv1_sub_descriptor->extradata_size = size;
        avio_read(pb, ffv1_sub_descriptor->extradata, size);
    }

    return 0;
}

static int mxf_read_indirect_value(void *arg, AVIOContext *pb, int size)
{
    MXFTaggedValue *tagged_value = arg;
    uint8_t key[17];
    int ret;

    if (size <= 17)
        return 0;

    ret = ffio_read_size(pb, key, 17);
    if (ret < 0)
        return ret;
    /* TODO: handle other types of of indirect values */
    if (memcmp(key, mxf_indirect_value_utf16le, 17) == 0) {
        return mxf_read_utf16le_string(pb, size - 17, &tagged_value->value);
    } else if (memcmp(key, mxf_indirect_value_utf16be, 17) == 0) {
        return mxf_read_utf16be_string(pb, size - 17, &tagged_value->value);
    }
    return 0;
}

static int mxf_read_tagged_value(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFTaggedValue *tagged_value = arg;
    switch (tag){
    case 0x5001:
        return mxf_read_utf16be_string(pb, size, &tagged_value->name);
    case 0x5003:
        return mxf_read_indirect_value(tagged_value, pb, size);
    }
    return 0;
}

/*
 * Match an uid independently of the version byte and up to len common bytes
 * Returns: boolean
 */
static int mxf_match_uid(const UID key, const uint8_t uid_prefix[], int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if (i != 7 && key[i] != uid_prefix[i])
            return 0;
    }
    return 1;
}

/* the essence element keys mxf_read_packet() dispatches on */
static int mxf_is_essence_element_key(const UID key)
{
    return mxf_match_uid(key, mxf_essence_element_key, 12) ||
           IS_KLV_KEY(key, mxf_canopus_essence_element_key) ||
           IS_KLV_KEY(key, mxf_avid_essence_element_key);
}

static const MXFCodecUL *mxf_get_codec_ul(const MXFCodecUL *uls, UID *uid)
{
    while (uls->uid[0]) {
        if(mxf_match_uid(uls->uid, *uid, uls->matching_len))
            break;
        uls++;
    }
    return uls;
}

static void *mxf_resolve_strong_ref(MXFContext *mxf, UID *strong_ref, enum MXFMetadataSetType type)
{
    MXFMetadataSet *mxf_metadata_set = NULL;
    MXFMetadataSet *best_mxf_metadata_set = NULL;
    MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[type];

    if (!strong_ref)
        return NULL;
    for (int i = mg->metadata_sets_count - 1; i >= 0; i--)
        if (!memcmp(*strong_ref, mg->metadata_sets[i]->uid, 16)) {
            // if type is Descriptor or Sequence and there are multiple matches, prefer one with duration set
            if (type != Descriptor && type != Sequence) {
                return mg->metadata_sets[i];
            }
            mxf_metadata_set = mg->metadata_sets[i];
            if (!best_mxf_metadata_set) {
                best_mxf_metadata_set = mxf_metadata_set;
            }
            else if (type == Descriptor) {
                MXFDescriptor *mxf_descriptor = (MXFDescriptor *)mxf_metadata_set;
                MXFDescriptor *best_mxf_descriptor = (MXFDescriptor *)best_mxf_metadata_set;
                if (best_mxf_descriptor->duration <= 0 && mxf_descriptor->duration > 0) {
                    best_mxf_metadata_set = mxf_metadata_set;
                }
            }
            else {
                MXFSequence *mxf_sequence = (MXFSequence *)mxf_metadata_set;
                MXFSequence *best_mxf_sequence = (MXFSequence *)best_mxf_metadata_set;
                if (best_mxf_sequence->duration <= 0 && mxf_sequence->duration > 0) {
                    best_mxf_metadata_set = mxf_metadata_set;
                }
            }
        }

    return best_mxf_metadata_set;;
}

static const MXFCodecUL mxf_picture_essence_container_uls[] = {
    // video essence container uls
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x07,0x0d,0x01,0x03,0x01,0x02,0x0c,0x01,0x00 }, 14,   AV_CODEC_ID_JPEG2000, NULL, 14, J2KWrap },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x10,0x60,0x01 }, 14,       AV_CODEC_ID_H264, NULL, 15 }, /* H.264 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x11,0x01,0x00 }, 14,      AV_CODEC_ID_DNXHD, NULL, 14 }, /* VC-3 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x0d,0x01,0x03,0x01,0x02,0x1e,0x01,0x00 }, 14,      AV_CODEC_ID_DNXUC, NULL, 14 }, /* DNxUncompressed / SMPTE RDD 50 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x12,0x01,0x00 }, 14,        AV_CODEC_ID_VC1, NULL, 14 }, /* VC-1 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x14,0x01,0x00 }, 14,       AV_CODEC_ID_TIFF, NULL, 14 }, /* TIFF */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x15,0x01,0x00 }, 14,      AV_CODEC_ID_DIRAC, NULL, 14 }, /* VC-2 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x1b,0x01,0x00 }, 14,       AV_CODEC_ID_CFHD, NULL, 14 }, /* VC-5 */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x1c,0x01,0x00 }, 14,     AV_CODEC_ID_PRORES, NULL, 14 }, /* ProRes */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x04,0x60,0x01 }, 14, AV_CODEC_ID_MPEG2VIDEO, NULL, 15 }, /* MPEG-ES */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x01,0x04,0x01 }, 14, AV_CODEC_ID_MPEG2VIDEO, NULL, 15, D10D11Wrap }, /* SMPTE D-10 mapping */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x0d,0x01,0x03,0x01,0x02,0x23,0x01,0x00 }, 14,       AV_CODEC_ID_FFV1, NULL, 14 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x02,0x41,0x01 }, 14,    AV_CODEC_ID_DVVIDEO, NULL, 15 }, /* DV 625 25mbps */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x05,0x00,0x00 }, 14,   AV_CODEC_ID_RAWVIDEO, NULL, 15, RawVWrap }, /* uncompressed picture */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0a,0x0e,0x0f,0x03,0x01,0x02,0x20,0x01,0x01 }, 15,     AV_CODEC_ID_HQ_HQA },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0a,0x0e,0x0f,0x03,0x01,0x02,0x20,0x02,0x01 }, 15,        AV_CODEC_ID_HQX },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0a,0x0e,0x15,0x00,0x04,0x02,0x10,0x00,0x01 }, 16,       AV_CODEC_ID_HEVC, NULL, 15 }, /* Canon XF-HEVC */
    { { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0xff,0x4b,0x46,0x41,0x41,0x00,0x0d,0x4d,0x4f }, 14,   AV_CODEC_ID_RAWVIDEO }, /* Legacy ?? Uncompressed Picture */
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0,      AV_CODEC_ID_NONE },
};

/* EC ULs for intra-only formats */
static const MXFCodecUL mxf_intra_only_essence_container_uls[] = {
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x01,0x00,0x00 }, 14, AV_CODEC_ID_MPEG2VIDEO }, /* MXF-GC SMPTE D-10 mappings */
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0,       AV_CODEC_ID_NONE },
};

/* intra-only PictureEssenceCoding ULs, where no corresponding EC UL exists */
static const MXFCodecUL mxf_intra_only_picture_essence_coding_uls[] = {
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0A,0x04,0x01,0x02,0x02,0x01,0x32,0x00,0x00 }, 14,       AV_CODEC_ID_H264 }, /* H.264/MPEG-4 AVC Intra Profiles */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x07,0x04,0x01,0x02,0x02,0x03,0x01,0x01,0x00 }, 14,   AV_CODEC_ID_JPEG2000 }, /* JPEG 2000 code stream */
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0,       AV_CODEC_ID_NONE },
};

/* actual coded width for AVC-Intra to allow selecting correct SPS/PPS */
static const MXFCodecUL mxf_intra_only_picture_coded_width[] = {
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0A,0x04,0x01,0x02,0x02,0x01,0x32,0x21,0x01 }, 16, 1440 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0A,0x04,0x01,0x02,0x02,0x01,0x32,0x21,0x02 }, 16, 1440 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0A,0x04,0x01,0x02,0x02,0x01,0x32,0x21,0x03 }, 16, 1440 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0A,0x04,0x01,0x02,0x02,0x01,0x32,0x21,0x04 }, 16, 1440 },
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0,    0 },
};

static const MXFCodecUL mxf_sound_essence_container_uls[] = {
    // sound essence container uls
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x06,0x01,0x00 }, 14, AV_CODEC_ID_PCM_S16LE, NULL, 14, RawAWrap }, /* BWF */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x02,0x0d,0x01,0x03,0x01,0x02,0x04,0x40,0x01 }, 14,       AV_CODEC_ID_MP2, NULL, 15 }, /* MPEG-ES */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x01,0x0d,0x01,0x03,0x01,0x02,0x01,0x01,0x01 }, 14, AV_CODEC_ID_PCM_S16LE, NULL, 13 }, /* D-10 Mapping 50Mbps PAL Extended Template */
    { { 0x06,0x0e,0x2b,0x34,0x01,0x01,0x01,0xff,0x4b,0x46,0x41,0x41,0x00,0x0d,0x4d,0x4F }, 14, AV_CODEC_ID_PCM_S16LE }, /* 0001GL00.MXF.A1.mxf_opatom.mxf */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x03,0x04,0x02,0x02,0x02,0x03,0x03,0x01,0x00 }, 14,       AV_CODEC_ID_AAC }, /* MPEG-2 AAC ADTS (legacy) */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x0d,0x01,0x03,0x01,0x02,0x16,0x00,0x00 }, 14,       AV_CODEC_ID_AAC, NULL, 14 }, /* AAC ADIF (SMPTE 381-4) */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x0d,0x01,0x03,0x01,0x02,0x17,0x00,0x00 }, 14,       AV_CODEC_ID_AAC, NULL, 14 }, /* AAC ADTS (SMPTE 381-4) */
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x0d,0x01,0x03,0x01,0x02,0x18,0x00,0x00 }, 14,       AV_CODEC_ID_AAC, NULL, 14 }, /* AAC LATM/LOAS (SMPTE 381-4) */
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0,      AV_CODEC_ID_NONE },
};

static const MXFCodecUL mxf_data_essence_container_uls[] = {
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x09,0x0d,0x01,0x03,0x01,0x02,0x0d,0x00,0x00 }, 16, AV_CODEC_ID_NONE,      "vbi_smpte_436M", 11 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x09,0x0d,0x01,0x03,0x01,0x02,0x0e,0x00,0x00 }, 16, AV_CODEC_ID_SMPTE_436M_ANC, "vbi_vanc_smpte_436M", 11 },
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x09,0x0d,0x01,0x03,0x01,0x02,0x13,0x01,0x01 }, 16, AV_CODEC_ID_TTML },
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },  0, AV_CODEC_ID_NONE },
};

typedef struct MXFChannelOrderingUL {
    UID uid;
    enum AVChannel channel;
    enum AVAudioServiceType service_type;
} MXFChannelOrderingUL;

static const MXFChannelOrderingUL mxf_channel_ordering[] = {
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x01,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_LEFT,            AV_AUDIO_SERVICE_TYPE_MAIN }, // Left
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x02,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_RIGHT,           AV_AUDIO_SERVICE_TYPE_MAIN }, // Right
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x03,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_CENTER,          AV_AUDIO_SERVICE_TYPE_MAIN }, // Center
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x04,0x00,0x00,0x00,0x00 }, AV_CHAN_LOW_FREQUENCY,         AV_AUDIO_SERVICE_TYPE_MAIN }, // Low Frequency Effects
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x05,0x00,0x00,0x00,0x00 }, AV_CHAN_SIDE_LEFT,             AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x06,0x00,0x00,0x00,0x00 }, AV_CHAN_SIDE_RIGHT,            AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x07,0x00,0x00,0x00,0x00 }, AV_CHAN_SIDE_SURROUND_LEFT,    AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Side Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x08,0x00,0x00,0x00,0x00 }, AV_CHAN_SIDE_SURROUND_RIGHT,   AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Side Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x09,0x00,0x00,0x00,0x00 }, AV_CHAN_BACK_LEFT,             AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Rear Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0a,0x00,0x00,0x00,0x00 }, AV_CHAN_BACK_RIGHT,            AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Rear Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0b,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_LEFT_OF_CENTER,  AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Center
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0c,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_RIGHT_OF_CENTER, AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Center
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0d,0x00,0x00,0x00,0x00 }, AV_CHAN_BACK_CENTER,           AV_AUDIO_SERVICE_TYPE_MAIN }, // Center Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0e,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_CENTER,          AV_AUDIO_SERVICE_TYPE_VISUALLY_IMPAIRED }, // Hearing impaired audio channel
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x0f,0x00,0x00,0x00,0x00 }, AV_CHAN_FRONT_CENTER,          AV_AUDIO_SERVICE_TYPE_HEARING_IMPAIRED }, // Visually impaired narrative audio channel
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x20,0x03,0x00,0x00,0x00 }, AV_CHAN_STEREO_LEFT,           AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Total
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x20,0x04,0x00,0x00,0x00 }, AV_CHAN_STEREO_RIGHT,          AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Total
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x01,0x00,0x00 }, AV_CHAN_TOP_FRONT_LEFT,        AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x02,0x00,0x00 }, AV_CHAN_TOP_FRONT_RIGHT,       AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x03,0x00,0x00 }, AV_CHAN_TOP_FRONT_CENTER,      AV_AUDIO_SERVICE_TYPE_MAIN }, // Center Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x04,0x00,0x00 }, AV_CHAN_TOP_SURROUND_LEFT,     AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x05,0x00,0x00 }, AV_CHAN_TOP_SURROUND_RIGHT,    AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x06,0x00,0x00 }, AV_CHAN_TOP_SIDE_LEFT,         AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Side Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x07,0x00,0x00 }, AV_CHAN_TOP_SIDE_RIGHT,        AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Side Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x08,0x00,0x00 }, AV_CHAN_TOP_BACK_LEFT,         AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Rear Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x09,0x00,0x00 }, AV_CHAN_TOP_BACK_RIGHT,        AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Rear Surround Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0a,0x00,0x00 }, AV_CHAN_TOP_SIDE_LEFT,         AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Top Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0b,0x00,0x00 }, AV_CHAN_TOP_SIDE_RIGHT,        AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Top Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0c,0x00,0x00 }, AV_CHAN_TOP_CENTER,            AV_AUDIO_SERVICE_TYPE_MAIN }, // Top Surround
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0d,0x00,0x00 }, AV_CHAN_LOW_FREQUENCY,         AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Front Subwoofer
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0e,0x00,0x00 }, AV_CHAN_LOW_FREQUENCY_2,       AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Front Subwoofer
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x0f,0x00,0x00 }, AV_CHAN_TOP_BACK_CENTER,       AV_AUDIO_SERVICE_TYPE_MAIN }, // Center Rear Height
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x10,0x00,0x00 }, AV_CHAN_BACK_CENTER,           AV_AUDIO_SERVICE_TYPE_MAIN }, // Center Rear
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x11,0x00,0x00 }, AV_CHAN_BOTTOM_FRONT_LEFT,     AV_AUDIO_SERVICE_TYPE_MAIN }, // Left Below
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x12,0x00,0x00 }, AV_CHAN_BOTTOM_FRONT_RIGHT,    AV_AUDIO_SERVICE_TYPE_MAIN }, // Right Below
    { { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x0d,0x03,0x02,0x01,0x30,0x01,0x13,0x00,0x00 }, AV_CHAN_BOTTOM_FRONT_CENTER,   AV_AUDIO_SERVICE_TYPE_MAIN }, // Center Below
    { { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }, 0,                             AV_AUDIO_SERVICE_TYPE_NB },
};

static MXFWrappingScheme mxf_get_wrapping_kind(UID *essence_container_ul)
{
    int val;
    const MXFCodecUL *codec_ul;

    codec_ul = mxf_get_codec_ul(mxf_picture_essence_container_uls, essence_container_ul);
    if (!codec_ul->uid[0])
        codec_ul = mxf_get_codec_ul(mxf_sound_essence_container_uls, essence_container_ul);
    if (!codec_ul->uid[0])
        codec_ul = mxf_get_codec_ul(mxf_data_essence_container_uls, essence_container_ul);
    if (!codec_ul->uid[0] || !codec_ul->wrapping_indicator_pos)
        return UnknownWrapped;

    val = (*essence_container_ul)[codec_ul->wrapping_indicator_pos];
    switch (codec_ul->wrapping_indicator_type) {
        case RawVWrap:
            val = val % 4;
            break;
        case RawAWrap:
            if (val == 0x03 || val == 0x04)
                val -= 0x02;
            break;
        case D10D11Wrap:
            if (val == 0x02)
                val = 0x01;
            break;
        case J2KWrap:
            if (val != 0x02)
                val = 0x01;
            break;
    }
    if (val == 0x01)
        return FrameWrapped;
    if (val == 0x02)
        return ClipWrapped;
    return UnknownWrapped;
}

static int mxf_get_sorted_table_segments(MXFContext *mxf, int *nb_sorted_segments, MXFIndexTableSegment ***sorted_segments)
{
    int i, j, nb_segments = 0;
    MXFIndexTableSegment **unsorted_segments;
    int last_body_sid = -1, last_index_sid = -1, last_index_start = -1;
    MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[IndexTableSegment];

    /* count number of segments, allocate arrays and copy unsorted segments */
    nb_segments = mg->metadata_sets_count;
    if (!nb_segments)
        return AVERROR_INVALIDDATA;

    if (!(unsorted_segments = av_calloc(nb_segments, sizeof(*unsorted_segments))) ||
        !(*sorted_segments  = av_calloc(nb_segments, sizeof(**sorted_segments)))) {
        av_freep(sorted_segments);
        av_free(unsorted_segments);
        return AVERROR(ENOMEM);
    }

    for (i = nb_segments = 0; i < mg->metadata_sets_count; i++) {
        MXFIndexTableSegment *s = (MXFIndexTableSegment*)mg->metadata_sets[i];
        if (s->edit_unit_byte_count || s->nb_index_entries)
            unsorted_segments[nb_segments++] = s;
        else
            av_log(mxf->fc, AV_LOG_WARNING, "IndexSID %i segment at %"PRId64" missing EditUnitByteCount and IndexEntryArray\n",
                   s->index_sid, s->index_start_position);
    }

    if (!nb_segments) {
        av_freep(sorted_segments);
        av_free(unsorted_segments);
        return AVERROR_INVALIDDATA;
    }

    *nb_sorted_segments = 0;

    /* sort segments by {BodySID, IndexSID, IndexStartPosition}, remove duplicates while we're at it */
    for (i = 0; i < nb_segments; i++) {
        int best = -1, best_body_sid = -1, best_index_sid = -1, best_index_start = -1;
        uint64_t best_index_duration = 0;

        for (j = 0; j < nb_segments; j++) {
            MXFIndexTableSegment *s = unsorted_segments[j];

            /* Require larger BosySID, IndexSID or IndexStartPosition then the previous entry. This removes duplicates.
             * We want the smallest values for the keys than what we currently have, unless this is the first such entry this time around.
             * If we come across an entry with the same IndexStartPosition but larger IndexDuration, then we'll prefer it over the one we currently have.
             */
            if ((i == 0 ||
                 s->body_sid >  last_body_sid ||
                 s->body_sid == last_body_sid && s->index_sid >  last_index_sid ||
                 s->body_sid == last_body_sid && s->index_sid == last_index_sid && s->index_start_position > last_index_start) &&
                (best == -1 ||
                 s->body_sid <  best_body_sid ||
                 s->body_sid == best_body_sid && s->index_sid <  best_index_sid ||
                 s->body_sid == best_body_sid && s->index_sid == best_index_sid && s->index_start_position <  best_index_start ||
                 s->body_sid == best_body_sid && s->index_sid == best_index_sid && s->index_start_position == best_index_start && s->index_duration > best_index_duration)) {
                best             = j;
                best_body_sid    = s->body_sid;
                best_index_sid   = s->index_sid;
                best_index_start = s->index_start_position;
                best_index_duration = s->index_duration;
            }
        }

        /* no suitable entry found -> we're done */
        if (best == -1)
            break;

        (*sorted_segments)[(*nb_sorted_segments)++] = unsorted_segments[best];
        last_body_sid    = best_body_sid;
        last_index_sid   = best_index_sid;
        last_index_start = best_index_start;
    }

    av_free(unsorted_segments);

    return 0;
}

/**
 * Computes the absolute file offset of the given essence container offset
 */
static int mxf_absolute_bodysid_offset(MXFContext *mxf, int body_sid, int64_t offset, int64_t *offset_out, MXFPartition **partition_out)
{
    MXFPartition *last_p = NULL;
    int a, b, m, m0;

    if (offset < 0)
        return AVERROR(EINVAL);

    a = -1;
    b = mxf->partitions_count;

    while (b - a > 1) {
        m0 = m = (a + b) >> 1;

        while (m < b && mxf->partitions[m].body_sid != body_sid)
            m++;

        if (m < b && mxf->partitions[m].body_offset <= offset)
            a = m;
        else
            b = m0;
    }

    if (a >= 0)
        last_p = &mxf->partitions[a];

    if (last_p && (!last_p->essence_length || last_p->essence_length > (offset - last_p->body_offset))) {
        *offset_out = last_p->essence_offset + (offset - last_p->body_offset);
        if (partition_out)
            *partition_out = last_p;
        return 0;
    }

    av_log(mxf->fc, AV_LOG_ERROR,
           "failed to find absolute offset of %"PRIX64" in BodySID %i - partial file?\n",
           offset, body_sid);

    return AVERROR_INVALIDDATA;
}

/**
 * Returns the end position of the essence container with given BodySID, or zero if unknown
 */
static int64_t mxf_essence_container_end(MXFContext *mxf, int body_sid)
{
    for (int x = mxf->partitions_count - 1; x >= 0; x--) {
        MXFPartition *p = &mxf->partitions[x];

        if (p->body_sid != body_sid)
            continue;

        if (!p->essence_length)
            return 0;

        return p->essence_offset + p->essence_length;
    }

    return 0;
}

/* EditUnit -> absolute offset */
static int mxf_edit_unit_absolute_offset(MXFContext *mxf, MXFIndexTable *index_table, int64_t edit_unit, AVRational edit_rate, int64_t *edit_unit_out, int64_t *offset_out, MXFPartition **partition_out, int nag)
{
    int i = 0, dir = 0;
    int64_t index_duration, index_end;
    MXFIndexTableSegment *first_segment, *last_segment;

    if (!index_table->nb_segments) {
        av_log(mxf->fc, AV_LOG_ERROR, "no index table segments\n");
        return AVERROR_INVALIDDATA;
    }

    edit_unit = av_rescale_q(edit_unit, index_table->segments[0]->index_edit_rate, edit_rate);

    first_segment = index_table->segments[0];
    last_segment  = index_table->segments[index_table->nb_segments - 1];

    // clamp to actual range of index
    index_end = av_sat_add64(last_segment->index_start_position, last_segment->index_duration);
    edit_unit = FFMAX(FFMIN(edit_unit, index_end), first_segment->index_start_position);
    if (edit_unit < 0)
        return AVERROR_PATCHWELCOME;

    // guess which table segment this edit unit is in
    // saturation is fine since it's just a guess
    // if the guess is wrong we revert to a linear search
    index_duration = av_sat_sub64(index_end, first_segment->index_start_position);

    // compute the guess, taking care not to cause overflow or division by zero
    if (index_duration > 0 && edit_unit <= INT64_MAX / index_table->nb_segments) {
        // a simple linear guesstimate
        // this is accurate to within +-1 when partitions are generated at a constant rate like mxfenc does
        int64_t i64 = index_table->nb_segments * edit_unit / index_duration;
        // clamp and downcast to 32-bit
        i = FFMAX(0, FFMIN(index_table->nb_segments - 1, i64));
    }

    for (; i >= 0 && i < index_table->nb_segments; i += dir) {
        MXFIndexTableSegment *s = index_table->segments[i];

        if (s->index_start_position <= edit_unit && edit_unit < s->index_start_position + s->index_duration) {
            int64_t index = edit_unit - s->index_start_position;
            int64_t offset_temp = s->offset;

            if (s->edit_unit_byte_count) {
                if (index > INT64_MAX / s->edit_unit_byte_count ||
                    s->edit_unit_byte_count * index > INT64_MAX - offset_temp)
                    return AVERROR_INVALIDDATA;

                offset_temp += s->edit_unit_byte_count * index;
            } else {
                if (s->nb_index_entries == 2 * s->index_duration + 1)
                    index *= 2;     /* Avid index */

                if (index < 0 || index >= s->nb_index_entries) {
                    av_log(mxf->fc, AV_LOG_ERROR, "IndexSID %i segment at %"PRId64" IndexEntryArray too small\n",
                           index_table->index_sid, s->index_start_position);
                    return AVERROR_INVALIDDATA;
                }

                offset_temp = s->stream_offset_entries[index];
            }

            if (edit_unit_out)
                *edit_unit_out = av_rescale_q(edit_unit, edit_rate, s->index_edit_rate);

            return mxf_absolute_bodysid_offset(mxf, index_table->body_sid, offset_temp, offset_out, partition_out);
        } else if (dir == 0) {
            // scan backwards if the segment is earlier than the current IndexStartPosition
            // else scan forwards
            if (edit_unit < s->index_start_position) {
                dir = -1;
            } else {
                dir = 1;
            }
        }
    }

    if (nag)
        av_log(mxf->fc, AV_LOG_ERROR, "failed to map EditUnit %"PRId64" in IndexSID %i to an offset\n", edit_unit, index_table->index_sid);

    return AVERROR_INVALIDDATA;
}

static int mxf_compute_ptses_fake_index(MXFContext *mxf, MXFIndexTable *index_table)
{
    int i, j, x;
    int8_t max_temporal_offset = -128;
    uint8_t *flags;

    /* first compute how many entries we have */
    for (i = 0; i < index_table->nb_segments; i++) {
        MXFIndexTableSegment *s = index_table->segments[i];

        if (!s->nb_index_entries) {
            index_table->nb_ptses = 0;
            return 0;                               /* no TemporalOffsets */
        }

        if (s->index_duration > INT_MAX - index_table->nb_ptses) {
            index_table->nb_ptses = 0;
            av_log(mxf->fc, AV_LOG_ERROR, "ignoring IndexSID %d, duration is too large\n", s->index_sid);
            return 0;
        }

        if (s->nb_index_entries != s->index_duration &&
            s->nb_index_entries != s->index_duration + 1 &&  /* Avid index */
            s->nb_index_entries != s->index_duration * 2 + 1) {
            index_table->nb_ptses = 0;
            av_log(mxf->fc, AV_LOG_ERROR, "ignoring IndexSID %d, duration does not match nb_index_entries\n", s->index_sid);
            return 0;
        }

        index_table->nb_ptses += s->index_duration;
    }

    /* paranoid check */
    if (index_table->nb_ptses <= 0)
        return 0;

    if (!(index_table->ptses      = av_malloc_array(index_table->nb_ptses, sizeof(int64_t))) ||
        !(index_table->fake_index = av_calloc(index_table->nb_ptses, sizeof(AVIndexEntry))) ||
        !(index_table->offsets    = av_malloc_array(index_table->nb_ptses, sizeof(int8_t))) ||
        !(flags                   = av_malloc_array(index_table->nb_ptses, sizeof(uint8_t)))) {
        av_freep(&index_table->ptses);
        av_freep(&index_table->fake_index);
        av_freep(&index_table->offsets);
        return AVERROR(ENOMEM);
    }

    /* we may have a few bad TemporalOffsets
     * make sure the corresponding PTSes don't have the bogus value 0 */
    for (x = 0; x < index_table->nb_ptses; x++)
        index_table->ptses[x] = AV_NOPTS_VALUE;

    /**
     * We have this:
     *
     * x  TemporalOffset
     * 0:  0
     * 1:  1
     * 2:  1
     * 3: -2
     * 4:  1
     * 5:  1
     * 6: -2
     *
     * We want to transform it into this:
     *
     * x  DTS PTS
     * 0: -1   0
     * 1:  0   3
     * 2:  1   1
     * 3:  2   2
     * 4:  3   6
     * 5:  4   4
     * 6:  5   5
     *
     * We do this by bucket sorting x by x+TemporalOffset[x] into mxf->ptses,
     * then settings ffstream(mxf)->first_dts = -max(TemporalOffset[x]).
     * The latter makes DTS <= PTS.
     */
    for (i = x = 0; i < index_table->nb_segments; i++) {
        MXFIndexTableSegment *s = index_table->segments[i];
        int index_delta = 1;
        int n = s->nb_index_entries;

        if (s->nb_index_entries == 2 * s->index_duration + 1)
            index_delta = 2;    /* Avid index */
        if (s->nb_index_entries == index_delta * s->index_duration + 1)
            /* ignore the last entry - it's the size of the essence container in Avid */
            n--;

        for (j = 0; j < n; j += index_delta, x++) {
            int offset = s->temporal_offset_entries[j] / index_delta;
            int index  = x + offset;

            if (x >= index_table->nb_ptses) {
                av_log(mxf->fc, AV_LOG_ERROR,
                       "x >= nb_ptses - IndexEntryCount %i < IndexDuration %"PRId64"?\n",
                       s->nb_index_entries, s->index_duration);
                break;
            }

            flags[x] = !(s->flag_entries[j] & 0x30) ? AVINDEX_KEYFRAME : 0;

            if (index < 0 || index >= index_table->nb_ptses) {
                av_log(mxf->fc, AV_LOG_ERROR,
                       "index entry %i + TemporalOffset %i = %i, which is out of bounds\n",
                       x, offset, index);
                continue;
            }

            index_table->offsets[x] = offset;
            index_table->ptses[index] = x;
            max_temporal_offset = FFMAX(max_temporal_offset, offset);
        }
    }

    /* calculate the fake index table in display order */
    for (x = 0; x < index_table->nb_ptses; x++) {
        index_table->fake_index[x].timestamp = x;
        if (index_table->ptses[x] != AV_NOPTS_VALUE)
            index_table->fake_index[index_table->ptses[x]].flags = flags[x];
    }
    av_freep(&flags);

    index_table->first_dts = -max_temporal_offset;

    return 0;
}

/**
 * Sorts and collects index table segments into index tables.
 * Also computes PTSes if possible.
 */
static int mxf_compute_index_tables(MXFContext *mxf)
{
    int ret, nb_sorted_segments;
    MXFIndexTableSegment **sorted_segments = NULL;

    if ((ret = mxf_get_sorted_table_segments(mxf, &nb_sorted_segments, &sorted_segments)) ||
        nb_sorted_segments <= 0) {
        av_log(mxf->fc, AV_LOG_WARNING, "broken or empty index\n");
        return 0;
    }

    /* sanity check and count unique BodySIDs/IndexSIDs */
    for (int i = 0; i < nb_sorted_segments; i++) {
        if (i == 0 || sorted_segments[i-1]->index_sid != sorted_segments[i]->index_sid)
            mxf->nb_index_tables++;
        else if (sorted_segments[i-1]->body_sid != sorted_segments[i]->body_sid) {
            av_log(mxf->fc, AV_LOG_ERROR, "found inconsistent BodySID\n");
            ret = AVERROR_INVALIDDATA;
            goto finish_decoding_index;
        }
    }

    mxf->index_tables = av_calloc(mxf->nb_index_tables,
                                  sizeof(*mxf->index_tables));
    if (!mxf->index_tables) {
        av_log(mxf->fc, AV_LOG_ERROR, "failed to allocate index tables\n");
        ret = AVERROR(ENOMEM);
        goto finish_decoding_index;
    }

    /* distribute sorted segments to index tables */
    for (int i = 0, j = 0; i < nb_sorted_segments; i++) {
        if (i != 0 && sorted_segments[i-1]->index_sid != sorted_segments[i]->index_sid) {
            /* next IndexSID */
            j++;
        }

        mxf->index_tables[j].nb_segments++;
    }

    for (int i = 0, j = 0; j < mxf->nb_index_tables; i += mxf->index_tables[j++].nb_segments) {
        MXFIndexTable *t = &mxf->index_tables[j];
        MXFTrack *mxf_track = NULL;
        int64_t offset_temp = 0;

        t->segments = av_calloc(t->nb_segments, sizeof(*t->segments));
        if (!t->segments) {
            av_log(mxf->fc, AV_LOG_ERROR, "failed to allocate IndexTableSegment"
                   " pointer array\n");
            ret = AVERROR(ENOMEM);
            goto finish_decoding_index;
        }

        if (sorted_segments[i]->index_start_position)
            av_log(mxf->fc, AV_LOG_WARNING, "IndexSID %i starts at EditUnit %"PRId64" - seeking may not work as expected\n",
                   sorted_segments[i]->index_sid, sorted_segments[i]->index_start_position);

        memcpy(t->segments, &sorted_segments[i], t->nb_segments * sizeof(MXFIndexTableSegment*));
        t->index_sid = sorted_segments[i]->index_sid;
        t->body_sid = sorted_segments[i]->body_sid;

        if ((ret = mxf_compute_ptses_fake_index(mxf, t)) < 0)
            goto finish_decoding_index;

        for (int k = 0; k < mxf->fc->nb_streams; k++) {
            MXFTrack *track = mxf->fc->streams[k]->priv_data;
            if (track && track->index_sid == t->index_sid) {
                mxf_track = track;
                break;
            }
        }

        /* fix zero IndexDurations and compute segment offsets */
        for (int k = 0; k < t->nb_segments; k++) {
            MXFIndexTableSegment *s = t->segments[k];

            if (!t->segments[k]->index_edit_rate.num || !t->segments[k]->index_edit_rate.den) {
                av_log(mxf->fc, AV_LOG_WARNING, "IndexSID %i segment %i has invalid IndexEditRate\n",
                       t->index_sid, k);
                if (mxf_track)
                    t->segments[k]->index_edit_rate = mxf_track->edit_rate;
            }

            s->offset = offset_temp;

            /* EditUnitByteCount == 0 for VBR indexes, which is fine since they use explicit StreamOffsets */
            if (s->edit_unit_byte_count && (s->index_duration > INT64_MAX / s->edit_unit_byte_count ||
                s->edit_unit_byte_count * s->index_duration > INT64_MAX - offset_temp)) {
                ret = AVERROR_INVALIDDATA;
                goto finish_decoding_index;
            }

            offset_temp += t->segments[k]->edit_unit_byte_count * t->segments[k]->index_duration;

            if (t->segments[k]->index_duration)
                continue;

            if (t->nb_segments > 1)
                av_log(mxf->fc, AV_LOG_WARNING, "IndexSID %i segment %i has zero IndexDuration and there's more than one segment\n",
                       t->index_sid, k);

            if (!mxf_track) {
                av_log(mxf->fc, AV_LOG_WARNING, "no streams?\n");
                break;
            }

            /* assume the first stream's duration is reasonable
             * leave index_duration = 0 on further segments in case we have any (unlikely)
             */
            t->segments[k]->index_duration = av_rescale_q(mxf_track->original_duration, t->segments[k]->index_edit_rate, mxf_track->edit_rate);
            break;
        }
    }

    ret = 0;
finish_decoding_index:
    av_free(sorted_segments);
    return ret;
}

static int mxf_is_st_422(const UID *essence_container_ul) {
    static const uint8_t st_422_essence_container_ul[] = { 0x06,0x0e,0x2b,0x34,0x04,0x01,0x01,0x07,0x0d,0x01,0x03,0x01,0x02,0x0c };

    return essence_container_ul && mxf_match_uid(*essence_container_ul, st_422_essence_container_ul,
                                                 sizeof(st_422_essence_container_ul));
}

static int mxf_is_intra_only(MXFDescriptor *descriptor)
{
    return mxf_get_codec_ul(mxf_intra_only_essence_container_uls,
                            &descriptor->essence_container_ul)->id != AV_CODEC_ID_NONE ||
           mxf_get_codec_ul(mxf_intra_only_picture_essence_coding_uls,
                            &descriptor->essence_codec_ul)->id     != AV_CODEC_ID_NONE;
}

static void mxf_umid_to_str(const UID ul, const UID uid,
                            char str[2 + sizeof(UID) * 4 + 1])
{
    snprintf(str, 2 + sizeof(UID) * 4 + 1, "0x");
    ff_data_to_hex(str + 2, ul, sizeof(UID), 0);
    ff_data_to_hex(str + 2 + 2 * sizeof(UID), uid, sizeof(UID), 0);
}

static int mxf_version_to_str(uint16_t major, uint16_t minor, uint16_t tertiary,
                              uint16_t patch, uint16_t release, char **str)
{
    *str = av_asprintf("%d.%d.%d.%d.%d", major, minor, tertiary, patch, release);
    if (!*str)
        return AVERROR(ENOMEM);
    return 0;
}

static int mxf_add_umid_metadata(AVDictionary **pm, const char *key, MXFPackage* package)
{
    char str[2 + 4 * sizeof(UID) + 1];
    if (!package)
        return 0;
    mxf_umid_to_str(package->package_ul, package->package_uid, str);
    av_dict_set(pm, key, str, 0);
    return 0;
}

static int mxf_add_timecode_metadata(AVDictionary **pm, const char *key, AVTimecode *tc)
{
    char buf[AV_TIMECODE_STR_SIZE];
    av_dict_set(pm, key, av_timecode_make_string(tc, buf, 0), 0);

    return 0;
}

static MXFTimecodeComponent* mxf_resolve_timecode_component(MXFContext *mxf, UID *strong_ref)
{
    MXFTimecodeComponent *timecode;
    MXFPulldownComponent *pulldown;

    timecode = mxf_resolve_strong_ref(mxf, strong_ref, TimecodeComponent);
    if (timecode)
        return timecode;

    pulldown = mxf_resolve_strong_ref(mxf, strong_ref, PulldownComponent);
    if (pulldown)
        return mxf_resolve_strong_ref(mxf, &pulldown->input_segment_ref, TimecodeComponent);

    return NULL;
}

static MXFPackage* mxf_resolve_source_package(MXFContext *mxf, UID package_ul, UID package_uid)
{
    MXFPackage *package = NULL;
    int i;

    for (i = 0; i < mxf->packages_count; i++) {
        package = mxf_resolve_strong_ref(mxf, &mxf->packages_refs[i], SourcePackage);
        if (!package)
            continue;

        if (!memcmp(package->package_ul, package_ul, 16) && !memcmp(package->package_uid, package_uid, 16))
            return package;
    }
    return NULL;
}

static MXFDescriptor* mxf_resolve_descriptor(MXFContext *mxf, UID *strong_ref, int track_id)
{
    MXFDescriptor *descriptor = mxf_resolve_strong_ref(mxf, strong_ref, Descriptor);
    if (descriptor)
        return descriptor;

    descriptor = mxf_resolve_strong_ref(mxf, strong_ref, MultipleDescriptor);
    if (descriptor) {
        for (int i = 0; i < descriptor->file_descriptors_count; i++) {
            MXFDescriptor *file_descriptor = mxf_resolve_strong_ref(mxf, &descriptor->file_descriptors_refs[i], Descriptor);

            if (!file_descriptor) {
                av_log(mxf->fc, AV_LOG_ERROR, "could not resolve file descriptor strong ref\n");
                continue;
            }
            if (file_descriptor->linked_track_id == track_id) {
                return file_descriptor;
            }
        }
    }

    return NULL;
}

static MXFStructuralComponent* mxf_resolve_sourceclip(MXFContext *mxf, UID *strong_ref)
{
    MXFStructuralComponent *component = NULL;
    MXFPackage *package = NULL;
    MXFDescriptor *descriptor = NULL;
    MXFEssenceGroup *essence_group;
    int i;

    component = mxf_resolve_strong_ref(mxf, strong_ref, SourceClip);
    if (component)
        return component;

    essence_group = mxf_resolve_strong_ref(mxf, strong_ref, EssenceGroup);
    if (!essence_group)
        return NULL;

    /* essence groups contains multiple representations of the same media,
       this return the first components with a valid Descriptor typically index 0 */
    for (i =0; i < essence_group->structural_components_count; i++){
        component = mxf_resolve_strong_ref(mxf, &essence_group->structural_components_refs[i], SourceClip);
        if (!component)
            continue;

        if (!(package = mxf_resolve_source_package(mxf, component->source_package_ul, component->source_package_uid)))
            continue;

        descriptor = mxf_resolve_strong_ref(mxf, &package->descriptor_ref, Descriptor);
        if (descriptor)
            return component;
    }

    return NULL;
}

static int mxf_parse_package_comments(MXFContext *mxf, AVDictionary **pm, MXFPackage *package)
{
    MXFTaggedValue *tag;
    int i;
    char *key = NULL;

    for (i = 0; i < package->comment_count; i++) {
        tag = mxf_resolve_strong_ref(mxf, &package->comment_refs[i], TaggedValue);
        if (!tag || !tag->name || !tag->value)
            continue;

        key = av_asprintf("comment_%s", tag->name);
        if (!key)
            return AVERROR(ENOMEM);

        av_dict_set(pm, key, tag->value, AV_DICT_DONT_STRDUP_KEY);
    }
    return 0;
}

static int mxf_parse_physical_source_package(MXFContext *mxf, MXFTrack *source_track, AVStream *st)
{
    MXFPackage *physical_package = NULL;
    MXFTrack *physical_track = NULL;
    MXFStructuralComponent *sourceclip = NULL;
    MXFTimecodeComponent *mxf_tc = NULL;
    int i, j, k;
    AVTimecode tc;
    int flags;
    int64_t start_position;

    for (i = 0; i < source_track->sequence->structural_components_count; i++) {
        sourceclip = mxf_resolve_strong_ref(mxf, &source_track->sequence->structural_components_refs[i], SourceClip);
        if (!sourceclip)
            continue;

        if (!(physical_package = mxf_resolve_source_package(mxf, sourceclip->source_package_ul, sourceclip->source_package_uid)))
            break;

        mxf_add_umid_metadata(&st->metadata, "reel_umid", physical_package);

        /* the name of physical source package is name of the reel or tape */
        if (physical_package->name && physical_package->name[0])
            av_dict_set(&st->metadata, "reel_name", physical_package->name, 0);

        /* the source timecode is calculated by adding the start_position of the sourceclip from the file source package track
         * to the start_frame of the timecode component located on one of the tracks of the physical source package.
         */
        for (j = 0; j < physical_package->tracks_count; j++) {
            if (!(physical_track = mxf_resolve_strong_ref(mxf, &physical_package->tracks_refs[j], Track))) {
                av_log(mxf->fc, AV_LOG_ERROR, "could not resolve source track strong ref\n");
                continue;
            }

            if (!(physical_track->sequence = mxf_resolve_strong_ref(mxf, &physical_track->sequence_ref, Sequence))) {
                av_log(mxf->fc, AV_LOG_ERROR, "could not resolve source track sequence strong ref\n");
                continue;
            }

            if (physical_track->edit_rate.num <= 0 ||
                physical_track->edit_rate.den <= 0) {
                av_log(mxf->fc, AV_LOG_WARNING,
                       "Invalid edit rate (%d/%d) found on structural"
                       " component #%d, defaulting to 25/1\n",
                       physical_track->edit_rate.num,
                       physical_track->edit_rate.den, i);
                physical_track->edit_rate = (AVRational){25, 1};
            }

            for (k = 0; k < physical_track->sequence->structural_components_count; k++) {
                if (!(mxf_tc = mxf_resolve_timecode_component(mxf, &physical_track->sequence->structural_components_refs[k])))
                    continue;

                flags = mxf_tc->drop_frame == 1 ? AV_TIMECODE_FLAG_DROPFRAME : 0;
                /* scale sourceclip start_position to match physical track edit rate */
                start_position = av_rescale_q(sourceclip->start_position,
                                              physical_track->edit_rate,
                                              source_track->edit_rate);

                if (av_sat_add64(start_position, mxf_tc->start_frame) != start_position + (uint64_t)mxf_tc->start_frame)
                    return AVERROR_INVALIDDATA;

                if (av_timecode_init(&tc, mxf_tc->rate, flags, start_position + mxf_tc->start_frame, mxf->fc) == 0) {
                    mxf_add_timecode_metadata(&st->metadata, "timecode", &tc);
                    return 0;
                }
            }
        }
    }

    return 0;
}

static int mxf_add_metadata_stream(MXFContext *mxf, MXFTrack *track)
{
    MXFStructuralComponent *component = NULL;
    const MXFCodecUL *codec_ul = NULL;
    MXFPackage tmp_package;
    AVStream *st;
    int j;

    for (j = 0; j < track->sequence->structural_components_count; j++) {
        component = mxf_resolve_sourceclip(mxf, &track->sequence->structural_components_refs[j]);
        if (!component)
            continue;
        break;
    }
    if (!component)
        return 0;

    st = avformat_new_stream(mxf->fc, NULL);
    if (!st) {
        av_log(mxf->fc, AV_LOG_ERROR, "could not allocate metadata stream\n");
        return AVERROR(ENOMEM);
    }

    st->codecpar->codec_type = AVMEDIA_TYPE_DATA;
    st->codecpar->codec_id = AV_CODEC_ID_NONE;
    st->id = track->track_id;

    memcpy(&tmp_package.package_ul, component->source_package_ul, 16);
    memcpy(&tmp_package.package_uid, component->source_package_uid, 16);
    mxf_add_umid_metadata(&st->metadata, "file_package_umid", &tmp_package);
    if (track->name && track->name[0])
        av_dict_set(&st->metadata, "track_name", track->name, 0);

    codec_ul = mxf_get_codec_ul(ff_mxf_data_definition_uls, &track->sequence->data_definition_ul);
    av_dict_set(&st->metadata, "data_type", av_get_media_type_string(codec_ul->id), 0);
    return 0;
}

static enum AVColorRange mxf_get_color_range(MXFContext *mxf, MXFDescriptor *descriptor)
{
    if (descriptor->black_ref_level || descriptor->white_ref_level || descriptor->color_range) {
        /* CDCI range metadata */
        if (!descriptor->component_depth)
            return AVCOL_RANGE_UNSPECIFIED;
        if (descriptor->black_ref_level == 0 && descriptor->component_depth < 31 &&
            descriptor->white_ref_level == ((1<<descriptor->component_depth) - 1) &&
            (descriptor->color_range    == (1<<descriptor->component_depth) ||
             descriptor->color_range    == ((1<<descriptor->component_depth) - 1)))
            return AVCOL_RANGE_JPEG;
        if (descriptor->component_depth >= 8 && descriptor->component_depth < 31 &&
            descriptor->black_ref_level == (1  <<(descriptor->component_depth - 4)) &&
            descriptor->white_ref_level == (235<<(descriptor->component_depth - 8)) &&
            descriptor->color_range     == ((14<<(descriptor->component_depth - 4)) + 1))
            return AVCOL_RANGE_MPEG;
        avpriv_request_sample(mxf->fc, "Unrecognized CDCI color range (color diff range %d, b %d, w %d, depth %d)",
                              descriptor->color_range, descriptor->black_ref_level,
                              descriptor->white_ref_level, descriptor->component_depth);
    }

    return AVCOL_RANGE_UNSPECIFIED;
}

static int is_pcm(enum AVCodecID codec_id)
{
    /* we only care about "normal" PCM codecs until we get samples */
    return codec_id >= AV_CODEC_ID_PCM_S16LE && codec_id < AV_CODEC_ID_PCM_S24DAUD;
}

static int set_language(AVFormatContext *s, const char *rfc5646, AVDictionary **met)
{
    // language abbr should contain at least 2 chars
    if (rfc5646 && strlen(rfc5646) > 1) {
        char primary_tag[4] =
            {rfc5646[0], rfc5646[1], rfc5646[2] != '-' ? rfc5646[2] : '\0', '\0'};

        const char *iso6392       = ff_convert_lang_to(primary_tag,
                                                       AV_LANG_ISO639_2_BIBL);
        if (iso6392)
            return(av_dict_set(met, "language", iso6392, 0));
    }
    return 0;
}

static MXFMCASubDescriptor *find_mca_link_id(MXFContext *mxf, enum MXFMetadataSetType type, UID *mca_link_id)
{
    MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[type];
    for (int k = 0; k < mg->metadata_sets_count; k++) {
        MXFMCASubDescriptor *group = (MXFMCASubDescriptor*)mg->metadata_sets[k];
        if (!memcmp(&group->mca_link_id, mca_link_id, 16))
            return group;
    }
    return NULL;
}

static void parse_ffv1_sub_descriptor(MXFContext *mxf, MXFTrack *source_track, MXFDescriptor *descriptor, AVStream *st)
{
    for (int i = 0; i < descriptor->sub_descriptors_count; i++) {
        MXFFFV1SubDescriptor *ffv1_sub_descriptor = mxf_resolve_strong_ref(mxf, &descriptor->sub_descriptors_refs[i], FFV1SubDescriptor);
        if (ffv1_sub_descriptor == NULL)
            continue;

        descriptor->extradata      = ffv1_sub_descriptor->extradata;
        descriptor->extradata_size = ffv1_sub_descriptor->extradata_size;
        ffv1_sub_descriptor->extradata = NULL;
        ffv1_sub_descriptor->extradata_size = 0;
        break;
    }
}

static int parse_mca_labels(MXFContext *mxf, MXFTrack *source_track, MXFDescriptor *descriptor, AVStream *st)
{
    AVChannelLayout *ch_layout = &st->codecpar->ch_layout;
    char *language = NULL;
    int ambigous_language = 0;
    enum AVAudioServiceType service_type = AV_AUDIO_SERVICE_TYPE_NB;
    int ambigous_service_type = 0;
    int ret;
    int channel_descriptors_invalid = 0;

    for (int i = 0; i < descriptor->sub_descriptors_count; i++) {
        char *channel_language;

        MXFMCASubDescriptor *label = mxf_resolve_strong_ref(mxf, &descriptor->sub_descriptors_refs[i], AudioChannelLabelSubDescriptor);
        if (label == NULL)
            continue;

        if (ch_layout->order == AV_CHANNEL_ORDER_UNSPEC) {
            av_channel_layout_uninit(ch_layout);
            ret = av_channel_layout_custom_init(ch_layout, descriptor->channels);
            if (ret < 0)
                return ret;
        }

        for (const MXFChannelOrderingUL* channel_ordering = mxf_channel_ordering; channel_ordering->uid[0]; channel_ordering++) {
            if (IS_KLV_KEY(channel_ordering->uid, label->mca_label_dictionary_id)) {
                int target_channel = label->mca_channel_id;
                if (target_channel == 0) {
                    if (descriptor->channels > 1) {
                        channel_descriptors_invalid = 1;
                    }
                    target_channel = 1;
                }
                if (target_channel <= 0 || target_channel > descriptor->channels) {
                    av_log(mxf->fc, AV_LOG_ERROR, "AudioChannelLabelSubDescriptor has invalid MCA channel ID %d\n", target_channel);
                    return AVERROR_INVALIDDATA;
                }
                ch_layout->u.map[target_channel - 1].id = channel_ordering->channel;
                if (service_type == AV_AUDIO_SERVICE_TYPE_NB)
                    service_type = channel_ordering->service_type;
                else if (service_type != channel_ordering->service_type)
                    ambigous_service_type = 1;
                break;
            }
        }

        channel_language = label->language;
        if (!channel_language) {
            MXFMCASubDescriptor *group = find_mca_link_id(mxf, SoundfieldGroupLabelSubDescriptor, &label->soundfield_group_link_id);
            if (group) {
                channel_language = group->language;
                if (!channel_language && group->group_of_soundfield_groups_link_id_count) {
                    MXFMCASubDescriptor *supergroup = find_mca_link_id(mxf, GroupOfSoundfieldGroupsLabelSubDescriptor,
                                                                       group->group_of_soundfield_groups_link_id_refs);
                    if (supergroup)
                        channel_language = supergroup->language;
                }
            }
        }
        if (channel_language) {
            if (language && strcmp(language, channel_language))
                ambigous_language = 1;
            else
                language = channel_language;
        }
    }

    if (language && !ambigous_language) {
       ret = set_language(mxf->fc, language, &st->metadata);
       if (ret < 0)
           return ret;
    }

    if (service_type != AV_AUDIO_SERVICE_TYPE_NB && service_type != AV_AUDIO_SERVICE_TYPE_MAIN && !ambigous_service_type) {
        enum AVAudioServiceType *ast;
        AVPacketSideData *side_data = av_packet_side_data_new(&st->codecpar->coded_side_data,
                                                              &st->codecpar->nb_coded_side_data,
                                                              AV_PKT_DATA_AUDIO_SERVICE_TYPE,
                                                              sizeof(*ast), 0);
        if (!side_data)
            return AVERROR(ENOMEM);
        ast = (enum AVAudioServiceType*)side_data->data;
        *ast = service_type;
    }

    ret = av_channel_layout_retype(ch_layout, 0, channel_descriptors_invalid ? 0 : AV_CHANNEL_LAYOUT_RETYPE_FLAG_CANONICAL);
    if (ret < 0)
        return ret;

    return 0;
}

static int mxf_parse_structural_metadata(MXFContext *mxf)
{
    MXFPackage *material_package = NULL;
    int k, ret;

    /* TODO: handle multiple material packages (OP3x) */
    for (int i = 0; i < mxf->packages_count; i++) {
        material_package = mxf_resolve_strong_ref(mxf, &mxf->packages_refs[i], MaterialPackage);
        if (material_package) break;
    }
    if (!material_package) {
        av_log(mxf->fc, AV_LOG_ERROR, "no material package found\n");
        return AVERROR_INVALIDDATA;
    }

    mxf_add_umid_metadata(&mxf->fc->metadata, "material_package_umid", material_package);
    if (material_package->name && material_package->name[0])
        av_dict_set(&mxf->fc->metadata, "material_package_name", material_package->name, 0);
    mxf_parse_package_comments(mxf, &mxf->fc->metadata, material_package);

    for (int i = 0; i < material_package->tracks_count; i++) {
        MXFPackage *source_package = NULL;
        MXFTrack *material_track = NULL;
        MXFTrack *source_track = NULL;
        MXFTrack *temp_track = NULL;
        MXFDescriptor *descriptor = NULL;
        MXFStructuralComponent *component = NULL;
        MXFTimecodeComponent *mxf_tc = NULL;
        UID *essence_container_ul = NULL;
        const MXFCodecUL *codec_ul = NULL;
        const MXFCodecUL *container_ul = NULL;
        const MXFCodecUL *pix_fmt_ul = NULL;
        AVStream *st;
        FFStream *sti;
        AVTimecode tc;
        int flags;

        if (!(material_track = mxf_resolve_strong_ref(mxf, &material_package->tracks_refs[i], Track))) {
            av_log(mxf->fc, AV_LOG_ERROR, "could not resolve material track strong ref\n");
            continue;
        }

        if ((component = mxf_resolve_strong_ref(mxf, &material_track->sequence_ref, TimecodeComponent))) {
            mxf_tc = (MXFTimecodeComponent*)component;
            flags = mxf_tc->drop_frame == 1 ? AV_TIMECODE_FLAG_DROPFRAME : 0;
            if (av_timecode_init(&tc, mxf_tc->rate, flags, mxf_tc->start_frame, mxf->fc) == 0) {
                mxf_add_timecode_metadata(&mxf->fc->metadata, "timecode", &tc);
            }
        }

        if (!(material_track->sequence = mxf_resolve_strong_ref(mxf, &material_track->sequence_ref, Sequence))) {
            av_log(mxf->fc, AV_LOG_ERROR, "could not resolve material track sequence strong ref\n");
            continue;
        }

        for (int j = 0; j < material_track->sequence->structural_components_count; j++) {
            component = mxf_resolve_strong_ref(mxf, &material_track->sequence->structural_components_refs[j], TimecodeComponent);
            if (!component)
                continue;

            mxf_tc = (MXFTimecodeComponent*)component;
            flags = mxf_tc->drop_frame == 1 ? AV_TIMECODE_FLAG_DROPFRAME : 0;
            if (av_timecode_init(&tc, mxf_tc->rate, flags, mxf_tc->start_frame, mxf->fc) == 0) {
                mxf_add_timecode_metadata(&mxf->fc->metadata, "timecode", &tc);
                break;
            }
        }

        /* TODO: handle multiple source clips, only finds first valid source clip */
        if(material_track->sequence->structural_components_count > 1)
            av_log(mxf->fc, AV_LOG_WARNING, "material track %d: has %d components\n",
                       material_track->track_id, material_track->sequence->structural_components_count);

        for (int j = 0; j < material_track->sequence->structural_components_count; j++) {
            component = mxf_resolve_sourceclip(mxf, &material_track->sequence->structural_components_refs[j]);
            if (!component)
                continue;

            source_package = mxf_resolve_source_package(mxf, component->source_package_ul, component->source_package_uid);
            if (!source_package) {
                av_log(mxf->fc, AV_LOG_TRACE, "material track %d: no corresponding source package found\n", material_track->track_id);
                continue;
            }
            for (k = 0; k < source_package->tracks_count; k++) {
                if (!(temp_track = mxf_resolve_strong_ref(mxf, &source_package->tracks_refs[k], Track))) {
                    av_log(mxf->fc, AV_LOG_ERROR, "could not resolve source track strong ref\n");
                    ret = AVERROR_INVALIDDATA;
                    goto fail_and_free;
                }
                if (temp_track->track_id == component->source_track_id) {
                    source_track = temp_track;
                    break;
                }
            }
            if (!source_track) {
                av_log(mxf->fc, AV_LOG_ERROR, "material track %d: no corresponding source track found\n", material_track->track_id);
                break;
            }

            for (k = 0; k < mxf->essence_container_data_count; k++) {
                MXFEssenceContainerData *essence_data;

                if (!(essence_data = mxf_resolve_strong_ref(mxf, &mxf->essence_container_data_refs[k], EssenceContainerData))) {
                    av_log(mxf->fc, AV_LOG_TRACE, "could not resolve essence container data strong ref\n");
                    continue;
                }
                if (!memcmp(component->source_package_ul, essence_data->package_ul, sizeof(UID)) && !memcmp(component->source_package_uid, essence_data->package_uid, sizeof(UID))) {
                    source_track->body_sid = essence_data->body_sid;
                    source_track->index_sid = essence_data->index_sid;
                    break;
                }
            }

            if(source_track && component)
                break;
        }
        if (!source_track || !component || !source_package) {
            if((ret = mxf_add_metadata_stream(mxf, material_track)))
                goto fail_and_free;
            continue;
        }

        if (!(source_track->sequence = mxf_resolve_strong_ref(mxf, &source_track->sequence_ref, Sequence))) {
            av_log(mxf->fc, AV_LOG_ERROR, "could not resolve source track sequence strong ref\n");
            ret = AVERROR_INVALIDDATA;
            goto fail_and_free;
        }

        /* 0001GL00.MXF.A1.mxf_opatom.mxf has the same SourcePackageID as 0001GL.MXF.V1.mxf_opatom.mxf
         * This would result in both files appearing to have two streams. Work around this by sanity checking DataDefinition */
        if (memcmp(material_track->sequence->data_definition_ul, source_track->sequence->data_definition_ul, 16)) {
            av_log(mxf->fc, AV_LOG_ERROR, "material track %d: DataDefinition mismatch\n", material_track->track_id);
            continue;
        }

        st = avformat_new_stream(mxf->fc, NULL);
        if (!st) {
            av_log(mxf->fc, AV_LOG_ERROR, "could not allocate stream\n");
            ret = AVERROR(ENOMEM);
            goto fail_and_free;
        }
        sti = ffstream(st);
        st->id = material_track->track_id;
        st->priv_data = source_track;

        descriptor = mxf_resolve_descriptor(mxf, &source_package->descriptor_ref, source_track->track_id);

        /* A SourceClip from a EssenceGroup may only be a single frame of essence data. The clips duration is then how many
         * frames its suppose to repeat for. Descriptor->duration, if present, contains the real duration of the essence data */
        if (descriptor && descriptor->duration != AV_NOPTS_VALUE) {
            if (component->duration > 0) {
                source_track->original_duration = st->duration = FFMIN(descriptor->duration, component->duration);
            }
            else if (descriptor->duration != AV_NOPTS_VALUE) {
                source_track->original_duration = st->duration = descriptor->duration;
            }
        }
        else
            source_track->original_duration = st->duration = component->duration;

        if (st->duration == -1) {
            if (material_track->sequence->duration > 0) {
                st->duration = material_track->sequence->duration;
            }
            else {
                st->duration = AV_NOPTS_VALUE;
            }
        }
        st->start_time = component->start_position;
        if (material_track->edit_rate.num <= 0 ||
            material_track->edit_rate.den <= 0) {
            av_log(mxf->fc, AV_LOG_WARNING,
                   "Invalid edit rate (%d/%d) found on stream #%d, "
                   "defaulting to 25/1\n",
                   material_track->edit_rate.num,
                   material_track->edit_rate.den, st->index);
            material_track->edit_rate = (AVRational){25, 1};
        }
        avpriv_set_pts_info(st, 64, material_track->edit_rate.den, material_track->edit_rate.num);

        /* ensure SourceTrack EditRate == MaterialTrack EditRate since only
         * the former is accessible via st->priv_data */
        source_track->edit_rate = material_track->edit_rate;

        PRINT_KEY(mxf->fc, "data definition   ul", source_track->sequence->data_definition_ul);
        codec_ul = mxf_get_codec_ul(ff_mxf_data_definition_uls, &source_track->sequence->data_definition_ul);
        st->codecpar->codec_type = codec_ul->id;

        if (!descriptor) {
            av_log(mxf->fc, AV_LOG_INFO, "source track %d: stream %d, no descriptor found\n", source_track->track_id, st->index);
            continue;
        }
        PRINT_KEY(mxf->fc, "essence codec     ul", descriptor->essence_codec_ul);
        PRINT_KEY(mxf->fc, "essence container ul", descriptor->essence_container_ul);
        essence_container_ul = &descriptor->essence_container_ul;
        source_track->wrapping = (mxf->op == OPAtom) ? ClipWrapped : mxf_get_wrapping_kind(essence_container_ul);
        if (source_track->wrapping == UnknownWrapped)
            av_log(mxf->fc, AV_LOG_INFO, "wrapping of stream %d is unknown\n", st->index);
        /* HACK: replacing the original key with mxf_encrypted_essence_container
         * is not allowed according to s429-6, try to find correct information anyway */
        if (IS_KLV_KEY(essence_container_ul, mxf_encrypted_essence_container)) {
            MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[CryptoContext];
            av_log(mxf->fc, AV_LOG_INFO, "broken encrypted mxf file\n");
            if (mg->metadata_sets_count) {
                MXFMetadataSet *metadata = mg->metadata_sets[0];
                essence_container_ul = &((MXFCryptoContext *)metadata)->source_container_ul;
            }
        }

        /* TODO: drop PictureEssenceCoding and SoundEssenceCompression, only check EssenceContainer */
        codec_ul = mxf_get_codec_ul(ff_mxf_codec_uls, &descriptor->essence_codec_ul);
        st->codecpar->codec_id = (enum AVCodecID)codec_ul->id;
        if (st->codecpar->codec_id == AV_CODEC_ID_NONE) {
            codec_ul = mxf_get_codec_ul(ff_mxf_codec_uls, &descriptor->codec_ul);
            st->codecpar->codec_id = (enum AVCodecID)codec_ul->id;
        }

        av_log(mxf->fc, AV_LOG_VERBOSE, "%s: Universal Label: ",
               avcodec_get_name(st->codecpar->codec_id));
        for (k = 0; k < 16; k++) {
            av_log(mxf->fc, AV_LOG_VERBOSE, "%.2x",
                   descriptor->essence_codec_ul[k]);
            if (!(k+1 & 19) || k == 5)
                av_log(mxf->fc, AV_LOG_VERBOSE, ".");
        }
        av_log(mxf->fc, AV_LOG_VERBOSE, "\n");

        mxf_add_umid_metadata(&st->metadata, "file_package_umid", source_package);
        if (source_package->name && source_package->name[0])
            av_dict_set(&st->metadata, "file_package_name", source_package->name, 0);
        if (material_track->name && material_track->name[0])
            av_dict_set(&st->metadata, "track_name", material_track->name, 0);

        mxf_parse_physical_source_package(mxf, source_track, st);

        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            source_track->intra_only = mxf_is_intra_only(descriptor);
            container_ul = mxf_get_codec_ul(mxf_picture_essence_container_uls, essence_container_ul);
            if (st->codecpar->codec_id == AV_CODEC_ID_NONE)
                st->codecpar->codec_id = container_ul->id;
            st->codecpar->width = descriptor->width;
            st->codecpar->height = descriptor->height; /* Field height, not frame height */
            switch (descriptor->frame_layout) {
                case FullFrame:
                    st->codecpar->field_order = AV_FIELD_PROGRESSIVE;
                    break;
                case OneField:
                    /* Every other line is stored and needs to be duplicated. */
                    av_log(mxf->fc, AV_LOG_INFO, "OneField frame layout isn't currently supported\n");
                    break; /* The correct thing to do here is fall through, but by breaking we might be
                              able to decode some streams at half the vertical resolution, rather than not al all.
                              It's also for compatibility with the old behavior. */
                case MixedFields:
                    break;
                case SegmentedFrame:
                    st->codecpar->field_order = AV_FIELD_PROGRESSIVE;
                case SeparateFields:
                    av_log(mxf->fc, AV_LOG_DEBUG, "video_line_map: (%d, %d), field_dominance: %d\n",
                           descriptor->video_line_map[0], descriptor->video_line_map[1],
                           descriptor->field_dominance);
                    if ((descriptor->video_line_map[0] > 0) && (descriptor->video_line_map[1] > 0)) {
                        /* Detect coded field order from VideoLineMap:
                         *  (even, even) => bottom field coded first
                         *  (even, odd)  => top field coded first
                         *  (odd, even)  => top field coded first
                         *  (odd, odd)   => bottom field coded first
                         */
                        if ((descriptor->video_line_map[0] + descriptor->video_line_map[1]) % 2) {
                            switch (descriptor->field_dominance) {
                                case MXF_FIELD_DOMINANCE_DEFAULT:
                                case MXF_FIELD_DOMINANCE_FF:
                                    st->codecpar->field_order = AV_FIELD_TT;
                                    break;
                                case MXF_FIELD_DOMINANCE_FL:
                                    st->codecpar->field_order = AV_FIELD_TB;
                                    break;
                                default:
                                    avpriv_request_sample(mxf->fc,
                                                          "Field dominance %d support",
                                                          descriptor->field_dominance);
                            }
                        } else {
                            switch (descriptor->field_dominance) {
                                case MXF_FIELD_DOMINANCE_DEFAULT:
                                case MXF_FIELD_DOMINANCE_FF:
                                    st->codecpar->field_order = AV_FIELD_BB;
                                    break;
                                case MXF_FIELD_DOMINANCE_FL:
                                    st->codecpar->field_order = AV_FIELD_BT;
                                    break;
                                default:
                                    avpriv_request_sample(mxf->fc,
                                                          "Field dominance %d support",
                                                          descriptor->field_dominance);
                            }
                        }
                    }
                    /* Turn field height into frame height. */
                    st->codecpar->height *= 2;
                    break;
                default:
                    av_log(mxf->fc, AV_LOG_INFO, "Unknown frame layout type: %d\n", descriptor->frame_layout);
            }

            if (mxf_is_st_422(essence_container_ul)) {
                switch ((*essence_container_ul)[14]) {
                case 2: /* Cn: Clip- wrapped Picture Element */
                case 3: /* I1: Interlaced Frame, 1 field/KLV */
                case 4: /* I2: Interlaced Frame, 2 fields/KLV */
                case 6: /* P1: Frame- wrapped Picture Element */
                    st->avg_frame_rate = source_track->edit_rate;
                    st->r_frame_rate = st->avg_frame_rate;
                    break;
                case 5: /* F1: Field-wrapped Picture Element */
                    st->avg_frame_rate = av_mul_q(av_make_q(2, 1), source_track->edit_rate);
                    st->r_frame_rate = st->avg_frame_rate;
                    break;
                default:
                    break;
                }
            }

            if (st->codecpar->codec_id == AV_CODEC_ID_PRORES) {
                switch (descriptor->essence_codec_ul[14]) {
                case 1: st->codecpar->codec_tag = MKTAG('a','p','c','o'); break;
                case 2: st->codecpar->codec_tag = MKTAG('a','p','c','s'); break;
                case 3: st->codecpar->codec_tag = MKTAG('a','p','c','n'); break;
                case 4: st->codecpar->codec_tag = MKTAG('a','p','c','h'); break;
                case 5: st->codecpar->codec_tag = MKTAG('a','p','4','h'); break;
                case 6: st->codecpar->codec_tag = MKTAG('a','p','4','x'); break;
                }
            }

            if (st->codecpar->codec_id == AV_CODEC_ID_RAWVIDEO) {
                st->codecpar->format = descriptor->pix_fmt;
                if (st->codecpar->format == AV_PIX_FMT_NONE) {
                    pix_fmt_ul = mxf_get_codec_ul(ff_mxf_pixel_format_uls,
                                                  &descriptor->essence_codec_ul);
                    st->codecpar->format = (enum AVPixelFormat)pix_fmt_ul->id;
                    if (st->codecpar->format== AV_PIX_FMT_NONE) {
                        st->codecpar->codec_tag = mxf_get_codec_ul(ff_mxf_codec_tag_uls,
                                                                   &descriptor->essence_codec_ul)->id;
                        if (!st->codecpar->codec_tag) {
                            /* support files created before RP224v10 by defaulting to UYVY422
                               if subsampling is 4:2:2 and component depth is 8-bit */
                            if (descriptor->horiz_subsampling == 2 &&
                                descriptor->vert_subsampling == 1 &&
                                descriptor->component_depth == 8) {
                                st->codecpar->format = AV_PIX_FMT_UYVY422;
                            }
                        }
                    }
                }
            }
            sti->need_parsing = AVSTREAM_PARSE_HEADERS;
            if (material_track->origin) {
                av_dict_set_int(&st->metadata, "material_track_origin", material_track->origin, 0);
            }
            if (source_track->origin) {
                av_dict_set_int(&st->metadata, "source_track_origin", source_track->origin, 0);
            }
            if (descriptor->aspect_ratio.num && descriptor->aspect_ratio.den)
                sti->display_aspect_ratio = descriptor->aspect_ratio;
            st->codecpar->color_range     = mxf_get_color_range(mxf, descriptor);
            st->codecpar->color_primaries = mxf_get_codec_ul(ff_mxf_color_primaries_uls, &descriptor->color_primaries_ul)->id;
            st->codecpar->color_trc       = mxf_get_codec_ul(ff_mxf_color_trc_uls, &descriptor->color_trc_ul)->id;
            st->codecpar->color_space     = mxf_get_codec_ul(ff_mxf_color_space_uls, &descriptor->color_space_ul)->id;
            if (descriptor->mastering) {
                if (!av_packet_side_data_add(&st->codecpar->coded_side_data, &st->codecpar->nb_coded_side_data,
                                             AV_PKT_DATA_MASTERING_DISPLAY_METADATA,
                                             (uint8_t *)descriptor->mastering, descriptor->mastering_size, 0)) {
                    ret = AVERROR(ENOMEM);
                    goto fail_and_free;
                }
                descriptor->mastering = NULL;
            }
            if (descriptor->coll) {
                if (!av_packet_side_data_add(&st->codecpar->coded_side_data, &st->codecpar->nb_coded_side_data,
                                             AV_PKT_DATA_CONTENT_LIGHT_LEVEL,
                                             (uint8_t *)descriptor->coll, descriptor->coll_size, 0)) {
                    ret = AVERROR(ENOMEM);
                    goto fail_and_free;
                }
                descriptor->coll = NULL;
            }

            for (int i = 0; i < descriptor->sub_descriptors_count; i++) {
                MXFAVCSubDescriptor *avc = mxf_resolve_strong_ref(mxf, &descriptor->sub_descriptors_refs[i], AVCSubDescriptor);
                if (avc == NULL)
                    continue;
                st->codecpar->bit_rate = avc->max_bit_rate;
            }
        } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            container_ul = mxf_get_codec_ul(mxf_sound_essence_container_uls, essence_container_ul);
            /* Only overwrite existing codec ID if it is unset or A-law, which is the default according to SMPTE RP 224. */
            if (st->codecpar->codec_id == AV_CODEC_ID_NONE || (st->codecpar->codec_id == AV_CODEC_ID_PCM_ALAW && (enum AVCodecID)container_ul->id != AV_CODEC_ID_NONE))
                st->codecpar->codec_id = (enum AVCodecID)container_ul->id;
            st->codecpar->ch_layout.nb_channels = descriptor->channels;

            if (descriptor->sample_rate.den > 0) {
                st->codecpar->sample_rate = descriptor->sample_rate.num / descriptor->sample_rate.den;
                avpriv_set_pts_info(st, 64, descriptor->sample_rate.den, descriptor->sample_rate.num);
            } else {
                av_log(mxf->fc, AV_LOG_WARNING, "invalid sample rate (%d/%d) "
                       "found for stream #%d, time base forced to 1/48000\n",
                       descriptor->sample_rate.num, descriptor->sample_rate.den,
                       st->index);
                avpriv_set_pts_info(st, 64, 1, 48000);
            }

            /* if duration is set, rescale it from EditRate to SampleRate */
            if (st->duration != AV_NOPTS_VALUE)
                st->duration = av_rescale_q(st->duration,
                                            av_inv_q(material_track->edit_rate),
                                            st->time_base);

            /* TODO: implement AV_CODEC_ID_RAWAUDIO */
            if (st->codecpar->codec_id == AV_CODEC_ID_PCM_S16LE) {
                if (descriptor->bits_per_sample > 16 && descriptor->bits_per_sample <= 24)
                    st->codecpar->codec_id = AV_CODEC_ID_PCM_S24LE;
                else if (descriptor->bits_per_sample == 32)
                    st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
            } else if (st->codecpar->codec_id == AV_CODEC_ID_PCM_S16BE) {
                if (descriptor->bits_per_sample > 16 && descriptor->bits_per_sample <= 24)
                    st->codecpar->codec_id = AV_CODEC_ID_PCM_S24BE;
                else if (descriptor->bits_per_sample == 32)
                    st->codecpar->codec_id = AV_CODEC_ID_PCM_S32BE;
            } else if (st->codecpar->codec_id == AV_CODEC_ID_MP2) {
                sti->need_parsing = AVSTREAM_PARSE_FULL;
            } else if (st->codecpar->codec_id == AV_CODEC_ID_AAC) {
                sti->need_parsing = AVSTREAM_PARSE_FULL;
            }
            st->codecpar->bits_per_coded_sample = av_get_bits_per_sample(st->codecpar->codec_id);

            if (descriptor->channels <= 0 || descriptor->channels >= FF_SANE_NB_CHANNELS) {
                av_log(mxf->fc, AV_LOG_ERROR, "Invalid number of channels %d, must be less than %d\n", descriptor->channels, FF_SANE_NB_CHANNELS);
                return AVERROR_INVALIDDATA;
            }

            ret = parse_mca_labels(mxf, source_track, descriptor, st);
            if (ret < 0)
                return ret;
        } else if (st->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
            enum AVMediaType type;
            container_ul = mxf_get_codec_ul(mxf_data_essence_container_uls, essence_container_ul);
            if (st->codecpar->codec_id == AV_CODEC_ID_NONE)
                st->codecpar->codec_id = container_ul->id;
            type = avcodec_get_type(st->codecpar->codec_id);
            if (type == AVMEDIA_TYPE_SUBTITLE)
                st->codecpar->codec_type = type;
            if (container_ul->desc)
                av_dict_set(&st->metadata, "data_type", container_ul->desc, 0);
            if (mxf->eia608_extract && st->codecpar->codec_id == AV_CODEC_ID_SMPTE_436M_ANC) {
                st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
                st->codecpar->codec_id = AV_CODEC_ID_EIA_608;
            }
        }
        if (!descriptor->extradata)
            parse_ffv1_sub_descriptor(mxf, source_track, descriptor, st);
        if (descriptor->extradata) {
            if (!ff_alloc_extradata(st->codecpar, descriptor->extradata_size)) {
                memcpy(st->codecpar->extradata, descriptor->extradata, descriptor->extradata_size);
            }
        } else if (st->codecpar->codec_id == AV_CODEC_ID_H264) {
            int coded_width = mxf_get_codec_ul(mxf_intra_only_picture_coded_width,
                                               &descriptor->essence_codec_ul)->id;
            if (coded_width)
                st->codecpar->width = coded_width;
            ret = ff_generate_avci_extradata(st);
            if (ret < 0)
                return ret;
        }
        if (st->codecpar->codec_type != AVMEDIA_TYPE_DATA && source_track->wrapping != FrameWrapped) {
            /* TODO: decode timestamps */
            sti->need_parsing = AVSTREAM_PARSE_TIMESTAMPS;
        }
    }

    for (int i = 0; i < mxf->fc->nb_streams; i++) {
        MXFTrack *track1 = mxf->fc->streams[i]->priv_data;
        if (track1 && track1->body_sid) {
            for (int j = i + 1; j < mxf->fc->nb_streams; j++) {
                MXFTrack *track2 = mxf->fc->streams[j]->priv_data;
                if (track2 && track1->body_sid == track2->body_sid && track1->wrapping != track2->wrapping) {
                    if (track1->wrapping == UnknownWrapped)
                        track1->wrapping = track2->wrapping;
                    else if (track2->wrapping == UnknownWrapped)
                        track2->wrapping = track1->wrapping;
                    else
                        av_log(mxf->fc, AV_LOG_ERROR, "stream %d and stream %d have the same BodySID (%d) "
                                                      "with different wrapping\n", i, j, track1->body_sid);
                }
            }
        }
    }

    ret = 0;
fail_and_free:
    return ret;
}

static int64_t mxf_timestamp_to_int64(uint64_t timestamp)
{
    struct tm time = { 0 };
    int msecs;
    time.tm_year = (timestamp >> 48) - 1900;
    time.tm_mon  = (timestamp >> 40 & 0xFF) - 1;
    time.tm_mday = (timestamp >> 32 & 0xFF);
    time.tm_hour = (timestamp >> 24 & 0xFF);
    time.tm_min  = (timestamp >> 16 & 0xFF);
    time.tm_sec  = (timestamp >> 8  & 0xFF);
    msecs        = (timestamp & 0xFF) * 4;

    /* Clip values for legacy reasons. Maybe we should return error instead? */
    time.tm_mon  = av_clip(time.tm_mon,  0, 11);
    time.tm_mday = av_clip(time.tm_mday, 1, 31);
    time.tm_hour = av_clip(time.tm_hour, 0, 23);
    time.tm_min  = av_clip(time.tm_min,  0, 59);
    time.tm_sec  = av_clip(time.tm_sec,  0, 59);
    msecs        = av_clip(msecs, 0, 999);

    return (int64_t)av_timegm(&time) * 1000000 + msecs * 1000;
}

#define SET_STR_METADATA(pb, name, str) do { \
    if ((ret = mxf_read_utf16be_string(pb, size, &str)) < 0) \
        return ret; \
    av_dict_set(&s->metadata, name, str, AV_DICT_DONT_STRDUP_VAL); \
} while (0)

#define SET_VERSION_METADATA(pb, name, major, minor, tertiary, patch, release, str) do { \
    major = avio_rb16(pb); \
    minor = avio_rb16(pb); \
    tertiary = avio_rb16(pb); \
    patch = avio_rb16(pb); \
    release = avio_rb16(pb); \
    if ((ret = mxf_version_to_str(major, minor, tertiary, patch, release, &str)) < 0) \
        return ret; \
    av_dict_set(&s->metadata, name, str, AV_DICT_DONT_STRDUP_VAL); \
} while (0)

#define SET_UID_METADATA(pb, name, var, str) do { \
    char uuid_str[2 * AV_UUID_LEN + 4 + 1]; \
    avio_read(pb, var, 16); \
    av_uuid_unparse(uid, uuid_str); \
    av_dict_set(&s->metadata, name, uuid_str, 0); \
} while (0)

#define SET_TS_METADATA(pb, name, var, str) do { \
    var = avio_rb64(pb); \
    if (var && (ret = ff_dict_set_timestamp(&s->metadata, name, mxf_timestamp_to_int64(var))) < 0) \
        return ret; \
} while (0)

static int mxf_read_identification_metadata(void *arg, AVIOContext *pb, int tag, int size, UID _uid, int64_t klv_offset)
{
    MXFContext *mxf = arg;
    AVFormatContext *s = mxf->fc;
    int ret;
    UID uid = { 0 };
    char *str = NULL;
    uint64_t ts;
    uint16_t major, minor, tertiary, patch, release;
    switch (tag) {
    case 0x3C01:
        SET_STR_METADATA(pb, "company_name", str);
        break;
    case 0x3C02:
        SET_STR_METADATA(pb, "product_name", str);
        break;
    case 0x3C03:
        SET_VERSION_METADATA(pb, "product_version_num", major, minor, tertiary, patch, release, str);
        break;
    case 0x3C04:
        SET_STR_METADATA(pb, "product_version", str);
        break;
    case 0x3C05:
        SET_UID_METADATA(pb, "product_uid", uid, str);
        break;
    case 0x3C06:
        SET_TS_METADATA(pb, "modification_date", ts, str);
        break;
    case 0x3C07:
        SET_VERSION_METADATA(pb, "toolkit_version_num", major, minor, tertiary, patch, release, str);
        break;
    case 0x3C08:
        SET_STR_METADATA(pb, "application_platform", str);
        break;
    case 0x3C09:
        SET_UID_METADATA(pb, "generation_uid", uid, str);
        break;
    case 0x3C0A:
        SET_UID_METADATA(pb, "uid", uid, str);
        break;
    }
    return 0;
}

static int mxf_read_preface_metadata(void *arg, AVIOContext *pb, int tag, int size, UID uid, int64_t klv_offset)
{
    MXFContext *mxf = arg;
    AVFormatContext *s = mxf->fc;
    int ret;
    char *str = NULL;

    if (tag >= 0x8000 && (IS_KLV_KEY(uid, mxf_avid_project_name))) {
        SET_STR_METADATA(pb, "project_name", str);
    }
    return 0;
}

static const MXFMetadataReadTableEntry mxf_metadata_read_table[] = {
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x05,0x01,0x00 }, mxf_read_primer_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x02,0x01,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x02,0x02,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x02,0x03,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x02,0x04,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x03,0x01,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x03,0x02,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x03,0x03,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x03,0x04,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x04,0x02,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x05,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x04,0x04,0x00 }, mxf_read_partition_pack },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x2f,0x00 }, mxf_read_preface_metadata },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x30,0x00 }, mxf_read_identification_metadata },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x18,0x00 }, mxf_read_content_storage },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x37,0x00 }, mxf_read_package, sizeof(MXFPackage), SourcePackage },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x36,0x00 }, mxf_read_package, sizeof(MXFPackage), MaterialPackage },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x0f,0x00 }, mxf_read_sequence, sizeof(MXFSequence), Sequence },
    { { 0x06,0x0E,0x2B,0x34,0x02,0x53,0x01,0x01,0x0D,0x01,0x01,0x01,0x01,0x01,0x05,0x00 }, mxf_read_essence_group, sizeof(MXFEssenceGroup), EssenceGroup},
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x11,0x00 }, mxf_read_source_clip, sizeof(MXFStructuralComponent), SourceClip },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x3f,0x00 }, mxf_read_tagged_value, sizeof(MXFTaggedValue), TaggedValue },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x44,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), MultipleDescriptor },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x42,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* Generic Sound */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x28,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* CDCI */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x29,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* RGBA */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x48,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* Wave */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x47,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* AES3 */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x51,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* MPEG2VideoDescriptor */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x5b,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* VBI - SMPTE 436M */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x5c,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* VANC/VBI - SMPTE 436M */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x5e,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* MPEG2AudioDescriptor */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x64,0x00 }, mxf_read_generic_descriptor, sizeof(MXFDescriptor), Descriptor }, /* DC Timed Text Descriptor */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x6b,0x00 }, mxf_read_mca_sub_descriptor, sizeof(MXFMCASubDescriptor), AudioChannelLabelSubDescriptor },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x6c,0x00 }, mxf_read_mca_sub_descriptor, sizeof(MXFMCASubDescriptor), SoundfieldGroupLabelSubDescriptor },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x6d,0x00 }, mxf_read_mca_sub_descriptor, sizeof(MXFMCASubDescriptor), GroupOfSoundfieldGroupsLabelSubDescriptor },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x81,0x03 }, mxf_read_ffv1_sub_descriptor, sizeof(MXFFFV1SubDescriptor), FFV1SubDescriptor },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x3A,0x00 }, mxf_read_track, sizeof(MXFTrack), Track }, /* Static Track */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x3B,0x00 }, mxf_read_track, sizeof(MXFTrack), Track }, /* Generic Track */
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x14,0x00 }, mxf_read_timecode_component, sizeof(MXFTimecodeComponent), TimecodeComponent },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x0c,0x00 }, mxf_read_pulldown_component, sizeof(MXFPulldownComponent), PulldownComponent },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x04,0x01,0x02,0x02,0x00,0x00 }, mxf_read_cryptographic_context, sizeof(MXFCryptoContext), CryptoContext },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x02,0x01,0x01,0x10,0x01,0x00 }, mxf_read_index_table_segment, sizeof(MXFIndexTableSegment), IndexTableSegment },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x23,0x00 }, mxf_read_essence_container_data, sizeof(MXFEssenceContainerData), EssenceContainerData },
    { { 0x06,0x0e,0x2b,0x34,0x02,0x53,0x01,0x01,0x0d,0x01,0x01,0x01,0x01,0x01,0x6e,0x00 }, mxf_read_avc_sub_descriptor, sizeof(MXFAVCSubDescriptor), AVCSubDescriptor }, /* AVC Sub-Descriptor */
    { { 0x06,0x0E,0x2B,0x34,0x01,0x01,0x01,0x02,0x03,0x01,0x02,0x10,0x01,0x00,0x00,0x00 } }, /* KLV fill, skip */
};

static int mxf_metadataset_init(MXFMetadataSet *ctx, enum MXFMetadataSetType type, MXFPartition *partition)
{
    ctx->partition_score = partition_score(partition);
    switch (type){
    case MultipleDescriptor:
    case Descriptor:
        ((MXFDescriptor*)ctx)->pix_fmt = AV_PIX_FMT_NONE;
        ((MXFDescriptor*)ctx)->duration = AV_NOPTS_VALUE;
        break;
    default:
        break;
    }
    return 0;
}

static int mxf_read_local_tags(MXFContext *mxf, KLVPacket *klv, MXFMetadataReadFunc *read_child, int ctx_size, enum MXFMetadataSetType type)
{
    AVIOContext *pb = mxf->fc->pb;
    uint64_t klv_end = avio_tell(pb) + klv->length;
    MXFMetadataSet *meta;
    void *ctx;

    if (ctx_size) {
        meta = av_mallocz(ctx_size);
        if (!meta)
            return AVERROR(ENOMEM);
        ctx  = meta;
        mxf_metadataset_init(meta, type, mxf->current_partition);
    } else {
        meta = NULL;
        ctx  = mxf;
    }
    while (avio_tell(pb) + 4ULL < klv_end && !avio_feof(pb)) {
        int ret;
        int tag = avio_rb16(pb);
        int size = avio_rb16(pb); /* KLV specified by 0x53 */
        int64_t next = avio_tell(pb);
        UID uid = {0};
        if (next < 0 || next > INT64_MAX - size) {
            if (meta) {
                mxf_free_metadataset(&meta, type);
            }
            return next < 0 ? next : AVERROR_INVALIDDATA;
        }
        next += size;

        av_log(mxf->fc, AV_LOG_TRACE, "local tag %#04x size %d\n", tag, size);
        if (!size) { /* ignore empty tag, needed for some files with empty UMID tag */
            av_log(mxf->fc, AV_LOG_ERROR, "local tag %#04x with 0 size\n", tag);
            continue;
        }
        if (tag > 0x7FFF) { /* dynamic tag */
            int i;
            for (i = 0; i < mxf->local_tags_count; i++) {
                int local_tag = AV_RB16(mxf->local_tags+i*18);
                if (local_tag == tag) {
                    memcpy(uid, mxf->local_tags+i*18+2, 16);
                    av_log(mxf->fc, AV_LOG_TRACE, "local tag %#04x\n", local_tag);
                    PRINT_KEY(mxf->fc, "uid", uid);
                }
            }
        }
        if (meta && tag == 0x3C0A) {
            avio_read(pb, meta->uid, 16);
        } else if ((ret = read_child(ctx, pb, tag, size, uid, -1)) < 0) {
            if (meta) {
                mxf_free_metadataset(&meta, type);
            }
            return ret;
        }

        /* Accept the 64k local set limit being exceeded (Avid). Don't accept
         * it extending past the end of the KLV though (zzuf5.mxf). */
        if (avio_tell(pb) > klv_end) {
            if (meta) {
                mxf_free_metadataset(&meta, type);
            }

            av_log(mxf->fc, AV_LOG_ERROR,
                   "local tag %#04x extends past end of local set @ %#"PRIx64"\n",
                   tag, klv->offset);
            return AVERROR_INVALIDDATA;
        } else if (avio_tell(pb) <= next)   /* only seek forward, else this can loop for a long time */
            avio_seek(pb, next, SEEK_SET);
    }
    return meta ? mxf_add_metadata_set(mxf, &meta, type) : 0;
}

/**
 * Matches any partition pack key, in other words:
 * - HeaderPartition
 * - BodyPartition
 * - FooterPartition
 * @return non-zero if the key is a partition pack key, zero otherwise
 */
static int mxf_is_partition_pack_key(UID key)
{
    //NOTE: this is a little lax since it doesn't constraint key[14]
    return !memcmp(key, mxf_header_partition_pack_key, 13) &&
            key[13] >= 2 && key[13] <= 4;
}

/**
 * Parses a metadata KLV
 * @return <0 on error, 0 otherwise
 */
static int mxf_parse_klv(MXFContext *mxf, KLVPacket klv, MXFMetadataReadFunc *read,
                                     int ctx_size, enum MXFMetadataSetType type)
{
    AVFormatContext *s = mxf->fc;
    int res;
    if (klv.key[5] == 0x53) {
        res = mxf_read_local_tags(mxf, &klv, read, ctx_size, type);
    } else {
        uint64_t next = avio_tell(s->pb) + klv.length;
        res = read(mxf, s->pb, 0, klv.length, klv.key, klv.offset);

        /* only seek forward, else this can loop for a long time */
        if (avio_tell(s->pb) > next) {
            av_log(s, AV_LOG_ERROR, "read past end of KLV @ %#"PRIx64"\n",
                   klv.offset);
            return AVERROR_INVALIDDATA;
        }

        avio_seek(s->pb, next, SEEK_SET);
    }
    if (res < 0) {
        av_log(s, AV_LOG_ERROR, "error reading header metadata\n");
        return res;
    }
    return 0;
}

/**
 * Seeks to the previous partition and parses it, if possible
 * @return <= 0 if we should stop parsing, > 0 if we should keep going
 */
static int mxf_seek_to_previous_partition(MXFContext *mxf)
{
    AVIOContext *pb = mxf->fc->pb;
    KLVPacket klv;
    int64_t current_partition_ofs;
    int ret;

    if (!mxf->current_partition ||
        mxf->run_in + mxf->current_partition->previous_partition <= mxf->last_forward_tell)
        return 0;   /* we've parsed all partitions */

    /* seek to previous partition */
    current_partition_ofs = mxf->current_partition->pack_ofs;   //includes run-in
    avio_seek(pb, mxf->run_in + mxf->current_partition->previous_partition, SEEK_SET);
    mxf->current_partition = NULL;

    av_log(mxf->fc, AV_LOG_TRACE, "seeking to previous partition\n");

    /* Make sure this is actually a PartitionPack, and if so parse it.
     * See deadlock2.mxf
     */
    if ((ret = klv_read_packet(mxf, &klv, pb)) < 0) {
        av_log(mxf->fc, AV_LOG_ERROR, "failed to read PartitionPack KLV\n");
        return ret;
    }

    if (!mxf_is_partition_pack_key(klv.key)) {
        av_log(mxf->fc, AV_LOG_ERROR, "PreviousPartition @ %" PRIx64 " isn't a PartitionPack\n", klv.offset);
        return AVERROR_INVALIDDATA;
    }

    /* We can't just check ofs >= current_partition_ofs because PreviousPartition
     * can point to just before the current partition, causing klv_read_packet()
     * to sync back up to it. See deadlock3.mxf
     */
    if (klv.offset >= current_partition_ofs) {
        av_log(mxf->fc, AV_LOG_ERROR, "PreviousPartition for PartitionPack @ %"
               PRIx64 " indirectly points to itself\n", current_partition_ofs);
        return AVERROR_INVALIDDATA;
    }

    if ((ret = mxf_parse_klv(mxf, klv, mxf_read_partition_pack, 0, 0)) < 0)
        return ret;

    return 1;
}

/**
 * Called when essence is encountered
 * @return <= 0 if we should stop parsing, > 0 if we should keep going
 */
static int mxf_parse_handle_essence(MXFContext *mxf)
{
    AVIOContext *pb = mxf->fc->pb;
    int64_t ret;

    if (mxf->parsing_backward) {
        if (mxf->skip_essence_parse)
            return 0;
        return mxf_seek_to_previous_partition(mxf);
    } else {
        if (!mxf->footer_partition) {
            av_log(mxf->fc, AV_LOG_TRACE, "no FooterPartition\n");
            return 0;
        }

        av_log(mxf->fc, AV_LOG_TRACE, "seeking to FooterPartition\n");

        /* remember where we were so we don't end up seeking further back than this */
        mxf->last_forward_tell = avio_tell(pb);

        if (!(pb->seekable & AVIO_SEEKABLE_NORMAL)) {
            av_log(mxf->fc, AV_LOG_INFO, "file is not seekable - not parsing FooterPartition\n");
            return -1;
        }

        /* seek to FooterPartition and parse backward */
        if ((ret = avio_seek(pb, mxf->run_in + mxf->footer_partition, SEEK_SET)) < 0) {
            av_log(mxf->fc, AV_LOG_ERROR,
                   "failed to seek to FooterPartition @ 0x%" PRIx64
                   " (%"PRId64") - partial file?\n",
                   mxf->run_in + mxf->footer_partition, ret);
            return ret;
        }

        mxf->current_partition = NULL;
        mxf->parsing_backward = 1;
    }

    return 1;
}

/**
 * Called when the next partition or EOF is encountered
 * @return <= 0 if we should stop parsing, > 0 if we should keep going
 */
static int mxf_parse_handle_partition_or_eof(MXFContext *mxf)
{
    return mxf->parsing_backward ? mxf_seek_to_previous_partition(mxf) : 1;
}

static MXFWrappingScheme mxf_get_wrapping_by_body_sid(AVFormatContext *s, int body_sid)
{
    for (int i = 0; i < s->nb_streams; i++) {
        MXFTrack *track = s->streams[i]->priv_data;
        if (track && track->body_sid == body_sid && track->wrapping != UnknownWrapped)
            return track->wrapping;
    }
    return UnknownWrapped;
}

/**
 * Figures out the proper offset and length of the essence container in each partition
 */
static void mxf_compute_essence_containers(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int x;

    for (x = 0; x < mxf->partitions_count; x++) {
        MXFPartition *p = &mxf->partitions[x];
        MXFWrappingScheme wrapping;

        if (!p->body_sid)
            continue;       /* BodySID == 0 -> no essence */

        /* for clip wrapped essences we point essence_offset after the KL (usually klv.offset + 20 or 25)
         * otherwise we point essence_offset at the key of the first essence KLV.
         */

        wrapping = (mxf->op == OPAtom) ? ClipWrapped : mxf_get_wrapping_by_body_sid(s, p->body_sid);

        if (wrapping == ClipWrapped) {
            p->essence_offset = p->first_essence_klv.next_klv - p->first_essence_klv.length;
            p->essence_length = p->first_essence_klv.length;
        } else {
            p->essence_offset = p->first_essence_klv.offset;

            /* essence container spans to the next partition */
            if (x < mxf->partitions_count - 1) {
                p->essence_length = mxf->partitions[x+1].pack_ofs - mxf->run_in - p->essence_offset;
            } else if (mxf->growing && x == mxf->partitions_count - 1) {
                /* growing: the current file size is the upper bound. No
                 * avio_tell() fallback - at this point the cursor is at
                 * essence_offset, so it would yield 0 dressed up as a computed
                 * length, and both mxf_absolute_bodysid_offset() and
                 * mxf_essence_container_end() already treat 0 as "unknown".
                 * mxf_growing_refresh_duration() keeps this up to date. */
                int64_t fs = avio_size(s->pb);
                p->essence_length = fs > p->essence_offset ? fs - p->essence_offset : 0;
            }

            if (p->essence_length < 0) {
                /* next ThisPartition < essence_offset */
                p->essence_length = 0;
                av_log(mxf->fc, AV_LOG_ERROR,
                       "partition %i: bad ThisPartition = %"PRIX64"\n",
                       x+1, mxf->partitions[x+1].pack_ofs - mxf->run_in);
            }
        }
    }
}

static MXFIndexTable *mxf_find_index_table(MXFContext *mxf, int index_sid)
{
    int i;
    for (i = 0; i < mxf->nb_index_tables; i++)
        if (mxf->index_tables[i].index_sid == index_sid)
            return &mxf->index_tables[i];
    return NULL;
}

/**
 * Deal with the case where for some audio atoms EditUnitByteCount is
 * very small (2, 4..). In those cases we should read more than one
 * sample per call to mxf_read_packet().
 */
static void mxf_compute_edit_units_per_packet(MXFContext *mxf, AVStream *st)
{
    MXFTrack *track = st->priv_data;
    MXFIndexTable *t;

    if (!track)
        return;
    track->edit_units_per_packet = 1;
    if (track->wrapping != ClipWrapped)
        return;

    t = mxf_find_index_table(mxf, track->index_sid);

    /* expect PCM with exactly one index table segment and a small (< 32) EUBC */
    if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO         ||
        !is_pcm(st->codecpar->codec_id)                        ||
        !t                                                     ||
        t->nb_segments != 1                                    ||
        t->segments[0]->edit_unit_byte_count >= 32)
        return;

    /* arbitrarily default to 48 kHz PAL audio frame size */
    /* TODO: We could compute this from the ratio between the audio
     *       and video edit rates for 48 kHz NTSC we could use the
     *       1802-1802-1802-1802-1801 pattern. */
    track->edit_units_per_packet = FFMAX(1, track->edit_rate.num / track->edit_rate.den / 25);
}

/**
 * Deal with the case where ClipWrapped essences does not have any IndexTableSegments.
 */
static int mxf_handle_missing_index_segment(MXFContext *mxf, AVStream *st)
{
    MXFTrack *track = st->priv_data;
    MXFIndexTableSegment *segment = NULL;
    MXFPartition *p = NULL;
    int essence_partition_count = 0;
    int edit_unit_byte_count = 0;
    int i, ret;
    MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[IndexTableSegment];

    if (!track || track->wrapping != ClipWrapped)
        return 0;

    /* check if track already has an IndexTableSegment */
    for (i = 0; i < mg->metadata_sets_count; i++) {
        MXFIndexTableSegment *s = (MXFIndexTableSegment*)mg->metadata_sets[i];
        if (s->body_sid == track->body_sid)
            return 0;
    }

    /* find the essence partition */
    for (i = 0; i < mxf->partitions_count; i++) {
        /* BodySID == 0 -> no essence */
        if (mxf->partitions[i].body_sid != track->body_sid)
            continue;

        p = &mxf->partitions[i];
        essence_partition_count++;
    }

    /* only handle files with a single essence partition */
    if (essence_partition_count != 1)
        return 0;

    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO && is_pcm(st->codecpar->codec_id)) {
        edit_unit_byte_count = (av_get_bits_per_sample(st->codecpar->codec_id) *
                                st->codecpar->ch_layout.nb_channels) >> 3;
    } else if (st->duration > 0 && p->first_essence_klv.length > 0 && p->first_essence_klv.length % st->duration == 0) {
        edit_unit_byte_count = p->first_essence_klv.length / st->duration;
    }

    if (edit_unit_byte_count <= 0)
        return 0;

    av_log(mxf->fc, AV_LOG_WARNING, "guessing index for stream %d using edit unit byte count %d\n", st->index, edit_unit_byte_count);

    if (!(segment = av_mallocz(sizeof(*segment))))
        return AVERROR(ENOMEM);

    if ((ret = mxf_add_metadata_set(mxf, (MXFMetadataSet**)&segment, IndexTableSegment)))
        return ret;

    /* Make sure we have nonzero unique index_sid, body_sid will be ok, because
     * using the same SID for index is forbidden in MXF. */
    if (!track->index_sid)
        track->index_sid = track->body_sid;

    /* stream will be treated as small EditUnitByteCount */
    segment->edit_unit_byte_count = edit_unit_byte_count;
    segment->index_start_position = 0;
    segment->index_duration = st->duration;
    segment->index_edit_rate = av_inv_q(st->time_base);
    segment->index_sid = track->index_sid;
    segment->body_sid = p->body_sid;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Growing (open) MXF support.
 *
 * A growing MXF is one still being written: it has no footer partition, and
 * usually an Open header partition (which is what ffmpeg's own muxer writes
 * for the whole life of the file - see mxfenc.c header_open_partition_key).
 * The header partition status is NOT a reliable growing indicator; the absence
 * of a footer is.
 *
 * In growing mode the demuxer reports a live duration, blocks at end-of-data
 * instead of returning EOF, resumes when the file grows, and optionally
 * maintains a sidecar index file so another process can read the current
 * duration with a single stat().
 *
 * Everything here is expressed in edit units of a single "reference track":
 * the first video stream, else the first audio stream, else stream 0.
 * ------------------------------------------------------------------------- */

#define MXF_GROWING_FOOTER_PROBE_US  1000000 /* footer probe cadence */
#define MXF_GROWING_SLEEP_SLICE_US    100000 /* interrupt responsiveness */
#define MXF_GROWING_STRIDE_SCAN_KLVS     256 /* bound on the stride walk */

#define MXF_GIDX_HEADER_SIZE  64
#define MXF_GIDX_ENTRY_SIZE   16
#define MXF_GIDX_FLAG_DENSE 0x01 /* entries are reference-track edit-unit dense */
#define MXF_GIDX_FLAG_KEY   0x40 /* SMPTE 377 random-access flag */

typedef enum MXFWaitResult {
    MXF_WAIT_RETRY = 0,   /* slept; re-test your predicate */
    MXF_WAIT_FINALIZED,   /* a footer exists; re-test once, then stop */
    MXF_WAIT_TIMEOUT,     /* growing_timeout_us elapsed since last progress */
    MXF_WAIT_INTERRUPT,   /* ff_check_interrupt() fired */
    MXF_WAIT_ERROR,       /* could not restore the AVIO position */
    MXF_WAIT_ROLE_CHANGED, /* reader took over as writer (or vice versa via a
                            * lost race); the AVIO position now sits at the
                            * new role's resume point, not the caller's saved
                            * position - only mxf_growing_reader_wait_step()
                            * returns this */
} MXFWaitResult;

static void mxf_read_random_index_pack(AVFormatContext *s);  /* forward decl */
static void mxf_growing_patch_sidecar_header(AVFormatContext *s);
static int  mxf_growing_reload_sidecar(AVFormatContext *s);
static void mxf_growing_patch_sidecar_duration(AVFormatContext *s, int64_t dur);
static int  mxf_growing_open_sidecar_read(AVFormatContext *s);
static int  mxf_growing_open_sidecar_write(AVFormatContext *s, int start_fresh);
static int  mxf_growing_probe_footer(AVFormatContext *s);
static int  mxf_growing_reader_wait_step(AVFormatContext *s);

static MXFTrack *mxf_growing_ref_track(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;

    if (mxf->growing_ref_stream < 0 || mxf->growing_ref_stream >= s->nb_streams)
        return NULL;
    return s->streams[mxf->growing_ref_stream]->priv_data;
}

static void mxf_growing_note_progress(MXFContext *mxf)
{
    mxf->growing_last_progress_us = av_gettime_relative();
}

/**
 * Pick the reference track: first video stream, else first audio, else the
 * first stream with a track attached.
 */
static void mxf_growing_select_ref_stream(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    static const enum AVMediaType want[2] = { AVMEDIA_TYPE_VIDEO, AVMEDIA_TYPE_AUDIO };

    mxf->growing_ref_stream = -1;
    for (int w = 0; w < 2; w++) {
        for (int i = 0; i < s->nb_streams; i++) {
            if (s->streams[i]->priv_data &&
                s->streams[i]->codecpar->codec_type == want[w]) {
                mxf->growing_ref_stream = i;
                return;
            }
        }
    }
    for (int i = 0; i < s->nb_streams; i++) {
        if (s->streams[i]->priv_data) {
            mxf->growing_ref_stream = i;
            return;
        }
    }
}

/**
 * Measure the byte stride of one edit unit of the reference track and record
 * the offset of its first essence element.
 *
 * Walks essence KLVs forward from scan_from, accepting only elements of the
 * reference stream, so KLV fill items, system items, index table segments and
 * other tracks' elements are all skipped. The AVIO position is restored on
 * every exit path.
 */
static int mxf_growing_measure_stride(AVFormatContext *s, int64_t scan_from)
{
    MXFContext *mxf = s->priv_data;
    int64_t saved = avio_tell(s->pb);
    int64_t starts[3], sizes[3];
    int n = 0, scanned = 0;
    int ret = 0;

    /* only the writer measures the stride - a reader trusts the sidecar the
     * writer maintains, and this seeks (destroying the demux cursor if
     * called from anywhere but header-time setup) */
    if (mxf->growing_role != MXF_GROWING_ROLE_WRITER)
        return 0;

    if (avio_seek(s->pb, scan_from, SEEK_SET) < 0) {
        avio_seek(s->pb, saved, SEEK_SET);
        return AVERROR(EIO);
    }

    while (n < 3 && scanned++ < MXF_GROWING_STRIDE_SCAN_KLVS) {
        KLVPacket klv;

        if (klv_read_packet(mxf, &klv, s->pb) < 0)
            break;
        if (mxf_is_essence_element_key(klv.key)) {
            int body_sid = find_body_sid_by_absolute_offset(mxf, klv.offset);
            if (mxf_get_stream_index(s, &klv, body_sid) == mxf->growing_ref_stream) {
                starts[n] = klv.offset;
                sizes[n]  = klv.next_klv - klv.offset;
                n++;
            }
        }
        if (avio_seek(s->pb, klv.next_klv, SEEK_SET) < 0)
            break;
    }

    if (n > 0)
        mxf->growing_essence_offset = starts[0];

    if (n == 3) {
        int64_t s01 = starts[1] - starts[0];
        int64_t s12 = starts[2] - starts[1];

        mxf->growing_stride_done = 1;
        if (s01 > 0 && s01 == s12) {
            mxf->growing_stride    = s01;
            mxf->growing_elem_size = sizes[0];
            av_log(s, AV_LOG_DEBUG, "growing MXF: measured CBR stride %"PRId64
                   " (element %"PRId64" bytes) from 0x%"PRIx64"\n",
                   s01, sizes[0], starts[0]);
        } else {
            av_log(s, AV_LOG_VERBOSE, "growing MXF: reference track is not CBR "
                   "(stride %"PRId64" then %"PRId64"); duration will come from "
                   "the sidecar index\n", s01, s12);
        }
    }

    if (avio_seek(s->pb, saved, SEEK_SET) < 0)
        ret = AVERROR(EIO);
    return ret;
}

/**
 * Learn the reference track's stride from KLVs the read path is already
 * walking. Does no I/O and no seeking, so it is safe to call per packet - the
 * header-time measurement fails on a file that has fewer than three edit units
 * when it is opened.
 */
static void mxf_growing_observe_klv(AVFormatContext *s, KLVPacket *klv,
                                    int stream_index)
{
    MXFContext *mxf = s->priv_data;
    int n;

    if (mxf->growing_role != MXF_GROWING_ROLE_WRITER)
        return;
    if (mxf->growing_stride_done || stream_index != mxf->growing_ref_stream)
        return;

    n = mxf->growing_stride_nb_obs;
    if (n > 0 && klv->offset <= mxf->growing_stride_obs[n - 1])
        return;                     /* re-read after a seek: do not mix offsets */
    if (n >= 3)
        return;
    mxf->growing_stride_obs[n] = klv->offset;
    mxf->growing_stride_nb_obs = ++n;

    if (n == 3) {
        int64_t s01 = mxf->growing_stride_obs[1] - mxf->growing_stride_obs[0];
        int64_t s12 = mxf->growing_stride_obs[2] - mxf->growing_stride_obs[1];

        mxf->growing_stride_done = 1;
        if (s01 > 0 && s01 == s12) {
            mxf->growing_stride         = s01;
            mxf->growing_essence_offset = mxf->growing_stride_obs[0];
            mxf->growing_elem_size      = klv->next_klv - klv->offset;
            av_log(s, AV_LOG_DEBUG, "growing MXF: observed CBR stride %"PRId64"\n",
                   s01);
            mxf_growing_patch_sidecar_header(s);
        } else {
            av_log(s, AV_LOG_VERBOSE, "growing MXF: reference track is not CBR; "
                   "duration will come from the sidecar index\n");
        }
    }
}

/**
 * Number of complete reference-track edit units currently present.
 * Returns AV_NOPTS_VALUE when it cannot be determined.
 *
 * Note there is deliberately no avio_tell() fallback when avio_size() is
 * unavailable: as a seek-availability metric that would report "available ==
 * already consumed" and make every forward seek block forever.
 */
static int64_t mxf_growing_compute_duration(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t fs, avail;

    /* a reader never derives duration from avio_size()/stride arithmetic -
     * the sidecar, maintained by the writer, is the sole source of truth;
     * see mxf_growing_available_edit_units(). */
    if (mxf->growing_role == MXF_GROWING_ROLE_READER)
        return AV_NOPTS_VALUE;

    if (mxf->growing_stride <= 0 || mxf->growing_elem_size <= 0)
        return AV_NOPTS_VALUE;

    fs = avio_size(s->pb);
    if (fs <= 0)
        return AV_NOPTS_VALUE;

    /* once the footer and RIP have landed, raw file size over-counts */
    if (mxf->growing_footer_offset > 0)
        fs = FFMIN(fs, mxf->run_in + mxf->growing_footer_offset);

    avail = fs - mxf->growing_essence_offset;
    if (avail < mxf->growing_elem_size)
        return 0;
    return (avail - mxf->growing_elem_size) / mxf->growing_stride + 1;
}

/**
 * How many reference-track edit units can be reached right now, taking the
 * best of the CBR geometry and the sidecar index (the latter is the only
 * source for VBR).
 */
static int64_t mxf_growing_available_edit_units(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t dur, idx = AV_NOPTS_VALUE;

    if (mxf->growing_role == MXF_GROWING_ROLE_READER) {
        /* the reader never re-derives duration itself: reload whatever the
         * writer has published in the sidecar (both the dense entries and
         * the O(1) duration field, which is what a clip-wrapped/header-only
         * sidecar has instead of entries) and trust that alone. */
        mxf_growing_reload_sidecar(s);
        if (mxf->growing_vbr_index && mxf->growing_vbr_index->nb_entries > 0)
            idx = mxf->growing_vbr_index->nb_entries;
        if (mxf->growing_sidecar_duration > 0)
            idx = idx == AV_NOPTS_VALUE ? mxf->growing_sidecar_duration
                                        : FFMAX(idx, mxf->growing_sidecar_duration);
        return idx;
    }

    dur = mxf_growing_compute_duration(s);
    if (mxf->growing_vbr_index) {
        mxf_growing_reload_sidecar(s);
        if (mxf->growing_vbr_index->nb_entries > 0)
            idx = mxf->growing_vbr_index->nb_entries;
    }
    if (dur == AV_NOPTS_VALUE) {
        /* idx stands as computed above */
    } else if (idx == AV_NOPTS_VALUE) {
        idx = dur;
    } else {
        idx = FFMAX(dur, idx);
    }

    /* HEVC and any other long-GOP codec get no bitstream-derived reorder
     * (see mxf_growing_index_ref_klv()/mxf_growing_set_reordered_pts()), so
     * the writer must not deliver past the last edit unit a real,
     * already-published container IndexTableSegment covers - that is the
     * only source mxf_set_pts() can correctly reorder from for such a track.
     * Costs up to one GOP of latency, always correct. Intra-only, MPEG-2 and
     * H.264 reference tracks are unaffected: they have their own reorder
     * source or nothing to reorder. */
    if (idx != AV_NOPTS_VALUE && mxf->growing_ref_stream >= 0 &&
        mxf->growing_ref_stream < s->nb_streams) {
        MXFTrack *ref_track = s->streams[mxf->growing_ref_stream]->priv_data;
        enum AVCodecID cid  = s->streams[mxf->growing_ref_stream]->codecpar->codec_id;

        if (!(ref_track && ref_track->intra_only) &&
            cid != AV_CODEC_ID_MPEG2VIDEO && cid != AV_CODEC_ID_H264) {
            MXFIndexTable *t = ref_track ? mxf_find_index_table(mxf, ref_track->index_sid) : NULL;

            idx = (t && t->nb_ptses > 0) ? FFMIN(idx, t->nb_ptses) : 0;
        }
    }

    return idx;
}

/**
 * Re-publish the live duration everywhere it is cached. Both
 * mxf_compute_index_tables() and mxf_compute_essence_containers() latch a
 * duration at open time, and mxf_edit_unit_absolute_offset() /
 * mxf_absolute_bodysid_offset() clamp against those latched values - so
 * without this a seek past the open-time duration fails even when the essence
 * is present.
 */
static void mxf_growing_refresh_duration(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    MXFTrack *ref = mxf_growing_ref_track(s);
    int64_t dur = mxf_growing_available_edit_units(s);
    int64_t fs;

    if (!ref || !ref->edit_rate.num || dur == AV_NOPTS_VALUE || dur <= 0)
        return;
    if (dur == mxf->growing_cur_duration)
        return;
    mxf->growing_cur_duration = dur;

    /* (1) per-track duration, rescaled out of reference edit units */
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MXFTrack *tr = st->priv_data;

        if (!tr || !tr->edit_rate.num)
            continue;
        tr->original_duration = av_rescale_q(dur, av_inv_q(ref->edit_rate),
                                             av_inv_q(tr->edit_rate));
        st->duration = av_rescale_q(dur, av_inv_q(ref->edit_rate), st->time_base);
    }

    /* (2) unfreeze mxf_edit_unit_absolute_offset()'s clamp. Only for a single
     *     CBR segment: a VBR segment's index_duration must keep matching
     *     nb_index_entries or mxf_compute_ptses_fake_index() breaks. */
    for (int j = 0; j < mxf->nb_index_tables; j++) {
        MXFIndexTable *t = &mxf->index_tables[j];

        if (t->nb_segments != 1 || !t->segments[0]->edit_unit_byte_count)
            continue;
        if (!t->segments[0]->index_edit_rate.num)
            continue;
        t->segments[0]->index_duration =
            av_rescale_q(dur, av_inv_q(ref->edit_rate),
                         av_inv_q(t->segments[0]->index_edit_rate));
    }

    /* (3) unfreeze mxf_absolute_bodysid_offset()'s bound. Never for
     *     clip-wrapped essence, whose KLV length is fixed. */
    fs = avio_size(s->pb);
    if (fs > 0 && mxf->partitions_count > 0) {
        MXFPartition *p = &mxf->partitions[mxf->partitions_count - 1];

        if (mxf->growing_footer_offset > 0)
            fs = FFMIN(fs, mxf->run_in + mxf->growing_footer_offset);
        if (p->body_sid && p->essence_offset > 0 && fs > p->essence_offset &&
            mxf_get_wrapping_by_body_sid(s, p->body_sid) != ClipWrapped)
            p->essence_length = fs - p->essence_offset;
    }

    /* (4) publish it in the sidecar header. A clip-wrapped reference track gets
     *     no entries at all - one KLV covers the whole essence, and for PCM the
     *     edit unit is a single sample, so per-edit-unit entries would run to
     *     gigabytes while carrying nothing the header does not already have. */
    mxf_growing_patch_sidecar_duration(s, dur);
}

/**
 * Look for evidence that the writer finalized the file. Position-safe on every
 * path. Never transitions - see the comment in mxf_read_packet().
 *
 * @return 1 if a footer partition was found and verified, 0 if not,
 *         AVERROR(EIO) if the AVIO position could not be restored.
 */
static int mxf_growing_probe_footer(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t saved = avio_tell(s->pb);
    uint64_t saved_fp = mxf->footer_partition;
    uint64_t cand = 0;
    int64_t fs;
    int ret = 0;

    if (mxf->growing_file_closed)
        return 1;
    if (!(s->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return 0;
    fs = avio_size(s->pb);
    if (fs <= 0)
        return 0;

    /* Signal 1: the RandomIndexPack at EOF. Self-validating - it matches the
     * full 16-byte RIP key and requires klv.next_klv == file_size - so it
     * cannot false-positive on essence bytes. mxf_write_footer() emits it
     * before it patches the header, so it appears first. */
    mxf_read_random_index_pack(s);
    if (mxf->footer_partition)
        cand = mxf->footer_partition;

    /* Signal 2: the header partition pack's FooterPartition field, at value
     * offset 24. Only present if the writer could seek back and patch it. */
    if (!cand && mxf->partitions_count > 0 &&
        mxf->partitions[0].pack_value_ofs > 0) {
        if (avio_seek(s->pb, mxf->partitions[0].pack_value_ofs + 24, SEEK_SET) >= 0) {
            uint64_t v = avio_rb64(s->pb);
            if (!avio_feof(s->pb) && v && mxf->run_in + (int64_t)v < fs)
                cand = v;
        }
    }

    /* Verify a Footer partition pack really is there. This is what turns two
     * heuristics into one deterministic signal, and it also catches a
     * half-rewritten header partition pack. */
    if (cand) {
        uint8_t key[16];

        if (avio_seek(s->pb, mxf->run_in + (int64_t)cand, SEEK_SET) >= 0 &&
            avio_read(s->pb, key, sizeof(key)) == sizeof(key) &&
            mxf_is_partition_pack_key(key) && key[13] == 4) {
            mxf->footer_partition      = cand;
            mxf->growing_footer_offset = cand;
            mxf->growing_file_closed   = 1;
            ret = 1;
            av_log(s, AV_LOG_INFO, "growing MXF: footer partition detected at "
                   "0x%"PRIx64"\n", mxf->run_in + (int64_t)cand);
        } else {
            /* do not leave an unverified value behind - mxf_read_partition_pack()
             * never clears a learned footer_partition */
            mxf->footer_partition = saved_fp;
        }
    }

    if (avio_seek(s->pb, saved, SEEK_SET) < 0)
        return AVERROR(EIO);
    return ret;
}

/**
 * True when the writer has not written as far as `end` yet.
 *
 * A growing file routinely ends in the middle of an edit unit, and
 * klv_read_packet() only needs the key and the BER length - both of which land
 * before the payload. Returning such a packet yields a short frame ("frame size
 * does not match index unit size"), so the read has to wait instead.
 *
 * `end` must be the end of the packet actually about to be returned: for
 * clip-wrapped essence that is the computed edit-unit sub-range, NOT
 * klv.next_klv, which is the end of the one KLV covering the whole essence and
 * therefore always lies beyond a growing file.
 *
 * The cached size keeps this off the fstat() path for every packet.
 */
static int mxf_growing_incomplete(AVFormatContext *s, int64_t end)
{
    MXFContext *mxf = s->priv_data;
    int64_t fs;

    if (end <= mxf->growing_last_size)
        return 0;
    fs = avio_size(s->pb);
    if (fs > 0)
        mxf->growing_last_size = fs;
    return end > mxf->growing_last_size;
}

/**
 * One iteration of the growing-mode wait, shared by mxf_read_packet()'s
 * end-of-data wait and mxf_read_seek()'s blocking wait.
 *
 * INVARIANT: on every return path the AVIO position is what it was on entry
 * and pb->eof_reached is clear (the restoring avio_seek() clears it
 * unconditionally). Callers may therefore retry a read at their own cursor
 * with no bookkeeping of their own.
 */
static int mxf_growing_wait_step(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t saved = avio_tell(s->pb);
    int64_t now, size, slept;
    int res = MXF_WAIT_RETRY;

    /* sticky states first: no sleep, no I/O */
    if (mxf->growing_timed_out)
        return MXF_WAIT_TIMEOUT;
    if (mxf->growing_file_closed)
        return MXF_WAIT_FINALIZED;
    if (ff_check_interrupt(&s->interrupt_callback))
        return MXF_WAIT_INTERRUPT;

    /* the stall timeout is checked before sleeping so it cannot overshoot by a
     * whole poll interval */
    now = av_gettime_relative();
    if (mxf->growing_timeout_us > 0 &&
        now - mxf->growing_last_progress_us >= mxf->growing_timeout_us) {
        av_log(s, AV_LOG_INFO, "growing MXF: no new data for %"PRId64" us, "
               "returning EOF\n", now - mxf->growing_last_progress_us);
        mxf->growing_timed_out = 1;
        av_dict_set(&s->metadata, "growing_timed_out", "1", 0);
        return MXF_WAIT_TIMEOUT;
    }

    /* sliced so a large growing_poll_us does not cost that much SIGINT latency */
    for (slept = 0; slept < mxf->growing_poll_us;
         slept += MXF_GROWING_SLEEP_SLICE_US) {
        av_usleep(FFMIN(MXF_GROWING_SLEEP_SLICE_US,
                        mxf->growing_poll_us - slept));
        if (ff_check_interrupt(&s->interrupt_callback))
            return MXF_WAIT_INTERRUPT;
    }

    /* any growth at all, even a partial edit unit, means the writer is alive */
    size = avio_size(s->pb);
    if (size > 0 && size > mxf->growing_last_size) {
        mxf->growing_last_size = size;
        mxf_growing_note_progress(mxf);
    }

    /* probe on a time base, not every Nth poll: growing_poll_us can be 1 ms */
    now = av_gettime_relative();
    if (now - mxf->growing_last_probe_us >= MXF_GROWING_FOOTER_PROBE_US) {
        mxf->growing_last_probe_us = now;
        if (mxf_growing_probe_footer(s) > 0)
            res = MXF_WAIT_FINALIZED;
    }

    if (avio_seek(s->pb, saved, SEEK_SET) < 0) {
        av_log(s, AV_LOG_ERROR, "growing MXF: cannot re-seek to %"PRId64"; the "
               "input does not support range requests\n", saved);
        return MXF_WAIT_ERROR;
    }
    return res;
}

/**
 * Rewind to `retry_pos` and wait for the file to grow.
 *
 * @return 1 to retry the read, 0 if the file finalized and the data will never
 *         arrive, or a negative AVERROR.
 */
static int mxf_growing_wait_for_data(AVFormatContext *s, int64_t retry_pos)
{
    MXFContext *mxf = s->priv_data;

    if (avio_seek(s->pb, retry_pos, SEEK_SET) < 0)
        return AVERROR(EIO);

    switch (mxf->growing_role == MXF_GROWING_ROLE_READER
            ? mxf_growing_reader_wait_step(s)
            : mxf_growing_wait_step(s)) {
    case MXF_WAIT_RETRY:
    case MXF_WAIT_ROLE_CHANGED: return 1;
    case MXF_WAIT_FINALIZED:    return 0;
    case MXF_WAIT_TIMEOUT:      return AVERROR_EOF;
    case MXF_WAIT_INTERRUPT:    return AVERROR_EXIT;
    default:                     return AVERROR(EIO);
    }
}

/* --- sidecar index ------------------------------------------------------- */

static void mxf_growing_release_sidecar_write(MXFContext *mxf)
{
    if (mxf->growing_index_out) {
        fclose(mxf->growing_index_out);
        mxf->growing_index_out = NULL;
    }
}

/**
 * Take an advisory write lock so a second process cannot interleave entries.
 * A failure is not fatal: the caller falls back to using the sidecar read-only.
 */
static int mxf_growing_sidecar_lock(AVFormatContext *s, FILE *f)
{
#if HAVE_FCNTL && HAVE_UNISTD_H
    struct flock fl;

    memset(&fl, 0, sizeof(fl));
    fl.l_type   = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start  = 0;
    fl.l_len    = 0;
    if (fcntl(fileno(f), F_SETLK, &fl) < 0)
        return AVERROR(errno);
#else
    av_log(s, AV_LOG_VERBOSE, "growing MXF: no fcntl(); the sidecar "
           "single-writer constraint is not enforced on this platform\n");
#endif
    return 0;
}

/**
 * A file that already has a footer at open builds no sidecar of its own (the
 * closed file's own container index already gives O(1) duration for free),
 * but a stale sidecar left behind by an earlier growing session of this same
 * output path must not linger and mislead a future reader into thinking the
 * file is still growing.
 *
 * Best-effort: try the write lock, delete the file if acquired, leave it
 * alone (logging only) if refused - deletion order between two processes
 * racing for the same stale file is inherently nondeterministic and is not
 * this function's problem to solve.
 */
static void mxf_growing_cleanup_stale_sidecar(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    FILE *f;

    f = avpriv_fopen_utf8(mxf->growing_index_file, "r+b");
    if (!f)
        return;                             /* nothing to clean up */

    if (mxf_growing_sidecar_lock(s, f) < 0) {
        av_log(s, AV_LOG_VERBOSE, "growing MXF: a stale-looking sidecar %s is "
               "locked by another process; leaving it for that process to "
               "clean up\n", mxf->growing_index_file);
        fclose(f);
        return;
    }
    fclose(f);   /* releases the lock - see the note on POSIX record locks */

    if (remove(mxf->growing_index_file) < 0)
        av_log(s, AV_LOG_WARNING, "growing MXF: could not remove stale "
               "sidecar %s: %s\n", mxf->growing_index_file,
               av_err2str(AVERROR(errno)));
    else
        av_log(s, AV_LOG_DEBUG, "growing MXF: removed stale sidecar %s\n",
               mxf->growing_index_file);
}

/**
 * Rewrite the header fields that are only known once the stride has been
 * measured. Possible because the handle is "r+b" and not append mode.
 */
static void mxf_growing_patch_sidecar_header(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    uint8_t buf[12];

    if (!mxf->growing_index_out)
        return;

    AV_WB64(buf + 0, mxf->growing_essence_offset);
    AV_WB32(buf + 8, mxf->growing_stride > 0 ? (uint32_t)mxf->growing_stride : 0);
    if (fseeko(mxf->growing_index_out, 16, SEEK_SET) == 0 &&
        fwrite(buf, 1, sizeof(buf), mxf->growing_index_out) == sizeof(buf))
        fflush(mxf->growing_index_out);
}

/**
 * Keep the header's duration field current. This is what a consumer reads for
 * an O(1) duration; the entry count only reflects what has been indexed.
 */
static void mxf_growing_patch_sidecar_duration(AVFormatContext *s, int64_t dur)
{
    MXFContext *mxf = s->priv_data;
    uint8_t buf[8];

    if (!mxf->growing_index_out || dur < 0)
        return;
    AV_WB64(buf, dur);
    if (fseeko(mxf->growing_index_out, 36, SEEK_SET) == 0 &&
        fwrite(buf, 1, sizeof(buf), mxf->growing_index_out) == sizeof(buf))
        fflush(mxf->growing_index_out);
}

/**
 * Open the sidecar for writing and take the write lock.
 *
 * MUST run after mxf_growing_open_sidecar_read() has closed its own handle: a
 * POSIX record lock is released when the process closes ANY descriptor to the
 * file.
 */
static int mxf_growing_open_sidecar_write(AVFormatContext *s, int start_fresh)
{
    MXFContext *mxf = s->priv_data;
    MXFTrack *ref = mxf_growing_ref_track(s);
    AVRational edit_rate = { 0, 1 };
    uint8_t hdr[MXF_GIDX_HEADER_SIZE];
    FILE *f;
    int64_t size;
    int ret;

    if (start_fresh) {
        f = avpriv_fopen_utf8(mxf->growing_index_file, "w+b");
    } else {
        f = avpriv_fopen_utf8(mxf->growing_index_file, "r+b");
        if (!f)
            f = avpriv_fopen_utf8(mxf->growing_index_file, "w+b");
    }
    if (!f) {
        av_log(s, AV_LOG_ERROR, "growing MXF: cannot open sidecar %s: %s\n",
               mxf->growing_index_file, av_err2str(AVERROR(errno)));
        return AVERROR(errno);
    }

    ret = mxf_growing_sidecar_lock(s, f);
    if (ret < 0) {
        av_log(s, AV_LOG_WARNING, "growing MXF: another process holds the "
               "sidecar write lock on %s (%s); using it read-only\n",
               mxf->growing_index_file, av_err2str(ret));
        fclose(f);
        return 0;
    }

    if (fseeko(f, 0, SEEK_END) < 0) {
        fclose(f);
        return AVERROR(EIO);
    }
    size = ftello(f);

    if (size == 0) {
        memset(hdr, 0, sizeof(hdr));
        memcpy(hdr, "MXFGIDX\x01", 8);
        hdr[8]  = 1;                    /* version */
        /* byte 9 is unused space between version and flags: this format has
         * no "complete" byte. Completion is signaled solely by the sidecar
         * file's deletion - see mxf_growing_transition_to_closed(). */
        hdr[10] = MXF_GIDX_FLAG_DENSE;  /* one entry per reference edit unit */
        if (ref)
            edit_rate = ref->edit_rate;
        AV_WB64(hdr + 16, mxf->growing_essence_offset);
        /* bytes per edit unit of the reference track, INCLUDING KLV overhead;
         * 0 means "not constant" (VBR) or "not yet measured" */
        AV_WB32(hdr + 24, mxf->growing_stride > 0 ? (uint32_t)mxf->growing_stride : 0);
        AV_WB32(hdr + 28, edit_rate.num);
        AV_WB32(hdr + 32, edit_rate.den);
        /* current duration in reference-track edit units, 0 = unknown. Kept up
         * to date by mxf_growing_patch_sidecar_duration(). This is the O(1)
         * duration a consumer should read: the entry count only tracks what has
         * been indexed, and clip-wrapped essence has no entries at all. */
        AV_WB64(hdr + 36, 0);
        if (fwrite(hdr, 1, sizeof(hdr), f) != sizeof(hdr) || fflush(f)) {
            av_log(s, AV_LOG_ERROR, "growing MXF: cannot write the sidecar "
                   "header to %s\n", mxf->growing_index_file);
            fclose(f);
            return AVERROR(EIO);
        }
        mxf->growing_index_entries_written = 0;
    } else if (size >= MXF_GIDX_HEADER_SIZE) {
        mxf->growing_index_entries_written =
            (size - MXF_GIDX_HEADER_SIZE) / MXF_GIDX_ENTRY_SIZE;
    } else {
        av_log(s, AV_LOG_ERROR, "growing MXF: sidecar %s is truncated "
               "(%"PRId64" bytes)\n", mxf->growing_index_file, size);
        fclose(f);
        return AVERROR_INVALIDDATA;
    }

    mxf->growing_index_out = f;
    av_log(s, AV_LOG_DEBUG, "growing MXF: sidecar %s open for writing, "
           "%"PRId64" existing entries\n", mxf->growing_index_file,
           mxf->growing_index_entries_written);
    return 0;
}

/**
 * Append one 16-byte entry. A torn or failed write disables the index rather
 * than leaving every later entry misaligned.
 */
static void mxf_growing_write_index_entry(AVFormatContext *s, int64_t offset,
                                          int32_t size, int8_t temporal_offset,
                                          uint8_t flags)
{
    MXFContext *mxf = s->priv_data;
    uint8_t e[MXF_GIDX_ENTRY_SIZE];

    if (!mxf->growing_index_out)
        return;

    AV_WB64(e + 0, offset);
    AV_WB32(e + 8, size);
    e[12] = (uint8_t)temporal_offset;
    e[13] = flags;
    e[14] = 0;
    e[15] = 0;

    /* the handle is "r+b", so the append position must be set explicitly.
     * fflush() per entry keeps a stat()-polling reader within one edit unit of
     * reality, which is the whole point of the sidecar. */
    if (fseeko(mxf->growing_index_out, 0, SEEK_END) < 0 ||
        fwrite(e, 1, sizeof(e), mxf->growing_index_out) != sizeof(e) ||
        fflush(mxf->growing_index_out)) {
        av_log(s, AV_LOG_ERROR, "growing MXF: sidecar write failed; disabling "
               "the index\n");
#if HAVE_UNISTD_H
        if (ftruncate(fileno(mxf->growing_index_out),
                      MXF_GIDX_HEADER_SIZE +
                      mxf->growing_index_entries_written * MXF_GIDX_ENTRY_SIZE) < 0)
            av_log(s, AV_LOG_WARNING, "growing MXF: could not truncate the torn "
                   "sidecar tail\n");
#endif
        mxf_growing_release_sidecar_write(mxf);
        mxf->growing_index_disabled = 1;
        return;
    }
    mxf->growing_index_entries_written++;
}

/**
 * Append one entry to the in-memory index, growing the arrays as needed.
 */
static int mxf_growing_vbr_append(MXFContext *mxf, int64_t offset, int32_t size,
                                  int8_t temporal_offset, uint8_t flags)
{
    MXFGrowingIndex *gi = mxf->growing_vbr_index;
    void *tmp;

    if (!gi)
        return 0;
    if (gi->nb_entries >= gi->entries_alloc) {
        int64_t new_alloc = FFMAX(gi->entries_alloc * 2, 1024);

        tmp = av_realloc_array(gi->offsets, new_alloc, sizeof(*gi->offsets));
        if (!tmp)
            return AVERROR(ENOMEM);
        gi->offsets = tmp;
        tmp = av_realloc_array(gi->sizes, new_alloc, sizeof(*gi->sizes));
        if (!tmp)
            return AVERROR(ENOMEM);
        gi->sizes = tmp;
        tmp = av_realloc_array(gi->temporal_offsets, new_alloc,
                               sizeof(*gi->temporal_offsets));
        if (!tmp)
            return AVERROR(ENOMEM);
        gi->temporal_offsets = tmp;
        tmp = av_realloc_array(gi->flags, new_alloc, sizeof(*gi->flags));
        if (!tmp)
            return AVERROR(ENOMEM);
        gi->flags = tmp;
        gi->entries_alloc = new_alloc;
    }
    gi->offsets[gi->nb_entries]          = offset;
    gi->sizes[gi->nb_entries]            = size;
    gi->temporal_offsets[gi->nb_entries] = temporal_offset;
    gi->flags[gi->nb_entries]            = flags;
    gi->nb_entries++;
    return 0;
}

static void mxf_growing_free_index(MXFContext *mxf)
{
    if (!mxf->growing_vbr_index)
        return;
    av_freep(&mxf->growing_vbr_index->offsets);
    av_freep(&mxf->growing_vbr_index->sizes);
    av_freep(&mxf->growing_vbr_index->temporal_offsets);
    av_freep(&mxf->growing_vbr_index->flags);
    av_freep(&mxf->growing_vbr_index);
}

/**
 * Load an existing sidecar into mxf->growing_vbr_index so this reader can
 * resume it rather than duplicating it.
 *
 * @return 0 if the sidecar is usable (possibly with zero entries),
 *         < 0 if it is absent, stale or incompatible - in which case the
 *         caller must start a fresh one.
 */
static int mxf_growing_open_sidecar_read(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    MXFTrack *ref = mxf_growing_ref_track(s);
    uint8_t hdr[MXF_GIDX_HEADER_SIZE];
    MXFGrowingIndex *gi;
    FILE *f;
    int64_t size, nb_entries, i;
    int ret;

    f = avpriv_fopen_utf8(mxf->growing_index_file, "rb");
    if (!f) {
        /* ENOENT: nothing exists yet, so "start fresh" cannot lose anything -
         * treat it the same as an invalid/unresumable sidecar. Anything else
         * (EACCES, EMFILE, ...) is transient and must not be mistaken for
         * grounds to recreate (and thereby truncate) a real, existing file. */
        return errno == ENOENT ? AVERROR_INVALIDDATA : AVERROR(errno);
    }

    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
        fclose(f);
        return AVERROR_INVALIDDATA;         /* empty or truncated: start fresh */
    }
    if (memcmp(hdr, "MXFGIDX\x01", 8) || hdr[8] != 1) {
        av_log(s, AV_LOG_WARNING, "growing MXF: sidecar %s has a bad magic or "
               "version; re-indexing\n", mxf->growing_index_file);
        fclose(f);
        return AVERROR_INVALIDDATA;
    }
    if (!(hdr[10] & MXF_GIDX_FLAG_DENSE)) {
        av_log(s, AV_LOG_WARNING, "growing MXF: sidecar %s is not edit-unit "
               "dense (written by an older build); re-indexing\n",
               mxf->growing_index_file);
        fclose(f);
        return AVERROR_INVALIDDATA;
    }
    if (ref && ref->edit_rate.num &&
        (AV_RB32(hdr + 28) != (uint32_t)ref->edit_rate.num ||
         AV_RB32(hdr + 32) != (uint32_t)ref->edit_rate.den)) {
        av_log(s, AV_LOG_WARNING, "growing MXF: sidecar %s was written for a "
               "different edit rate; re-indexing\n", mxf->growing_index_file);
        fclose(f);
        return AVERROR_INVALIDDATA;
    }

    /* a reader never measures these itself (mxf_growing_measure_stride()/
     * mxf_growing_observe_klv() are writer-only) - load them from the
     * incumbent writer's header so a later takeover with an empty in-memory
     * index (mxf_growing_attempt_takeover()) has a real fallback instead of
     * the zero-initialized default. */
    mxf->growing_essence_offset = AV_RB64(hdr + 16);
    mxf->growing_stride         = (int64_t)AV_RB32(hdr + 24);

    if (fseeko(f, 0, SEEK_END) < 0) {
        fclose(f);
        return AVERROR(EIO);
    }
    size = ftello(f);
    nb_entries = (size - MXF_GIDX_HEADER_SIZE) / MXF_GIDX_ENTRY_SIZE;
    if (nb_entries < 0)
        nb_entries = 0;

    gi = av_mallocz(sizeof(*gi));
    if (!gi) {
        fclose(f);
        return AVERROR(ENOMEM);
    }

    if (fseeko(f, MXF_GIDX_HEADER_SIZE, SEEK_SET) < 0) {
        av_freep(&gi);
        fclose(f);
        return AVERROR(EIO);
    }

    /* hand gi to the context up front so mxf_growing_vbr_append() can grow it */
    mxf_growing_free_index(mxf);
    mxf->growing_vbr_index = gi;

    for (i = 0; i < nb_entries; i++) {
        uint8_t e[MXF_GIDX_ENTRY_SIZE];

        if (fread(e, 1, sizeof(e), f) != sizeof(e))
            break;
        ret = mxf_growing_vbr_append(mxf, AV_RB64(e + 0), AV_RB32(e + 8),
                                     (int8_t)e[12], e[13]);
        if (ret < 0) {
            fclose(f);
            return ret;
        }
    }

    fclose(f);
    av_log(s, AV_LOG_DEBUG, "growing MXF: loaded %"PRId64" sidecar entries from "
           "%s\n", gi->nb_entries, mxf->growing_index_file);
    return 0;
}

/**
 * Pick up entries (and the published duration) another process has appended
 * since the last look.
 *
 * Deliberately a no-op when we hold the write lock: re-opening the path would
 * release this process's fcntl() record lock on the first close, and our own
 * in-memory index is authoritative anyway.
 *
 * @return 1 if the sidecar advanced (new entries and/or a larger published
 *         duration) - this IS the reader's index-stall progress signal,
 *         0 if it exists but did not advance,
 *         AVERROR(ENOENT) if the sidecar is gone (completion, or a stale
 *         index removed out from under us - the caller must tell those apart
 *         with mxf_growing_probe_footer()),
 *         another negative AVERROR on I/O failure.
 */
static int mxf_growing_reload_sidecar(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    MXFGrowingIndex *gi = mxf->growing_vbr_index;
    FILE *f;
    uint8_t durbuf[8];
    int64_t size, nb_on_disk, i, new_duration;
    int advanced = 0;

    if (mxf->growing_index_out)
        return 0;
    if (!mxf->growing_index_file || !gi)
        return 0;

    f = avpriv_fopen_utf8(mxf->growing_index_file, "rb");
    if (!f)
        return AVERROR(errno);

    if (fseeko(f, 0, SEEK_END) < 0) {
        fclose(f);
        return AVERROR(EIO);
    }
    size = ftello(f);
    if (size < MXF_GIDX_HEADER_SIZE) {
        fclose(f);
        return 0;                          /* still being created */
    }

    if (fseeko(f, 36, SEEK_SET) == 0 &&
        fread(durbuf, 1, sizeof(durbuf), f) == sizeof(durbuf)) {
        new_duration = (int64_t)AV_RB64(durbuf);
        if (new_duration > mxf->growing_sidecar_duration) {
            mxf->growing_sidecar_duration = new_duration;
            advanced = 1;
        }
    }

    nb_on_disk = (size - MXF_GIDX_HEADER_SIZE) / MXF_GIDX_ENTRY_SIZE;
    if (nb_on_disk > gi->nb_entries) {
        if (fseeko(f, MXF_GIDX_HEADER_SIZE + gi->nb_entries * MXF_GIDX_ENTRY_SIZE,
                   SEEK_SET) == 0) {
            for (i = gi->nb_entries; i < nb_on_disk; i++) {
                uint8_t e[MXF_GIDX_ENTRY_SIZE];

                if (fread(e, 1, sizeof(e), f) != sizeof(e))
                    break;
                if (mxf_growing_vbr_append(mxf, AV_RB64(e + 0), AV_RB32(e + 8),
                                           (int8_t)e[12], e[13]) < 0)
                    break;
            }
        }
        advanced = 1;
    }
    fclose(f);
    return advanced;
}

/**
 * Index one essence element of the reference track.
 *
 * Entry i must be edit unit i, so appending is only legal at the end of what
 * is already indexed. Rather than a byte high-water mark (which silently
 * permits a gap when the read starts mid-file), we recognise the last indexed
 * KLV by its recorded offset and resume from the one after it.
 */
#define MXF_GROWING_MPEG2_SCAN 256    /* group/picture header always lands well within this */
#define MXF_GROWING_H264_SCAN  16384  /* SPS/PPS/SEI + start of first slice NAL */

/**
 * MPEG-2: temporal_reference (10 bits) and picture_coding_type (3 bits) sit
 * immediately after picture_start_code (0x00000100) in the picture header;
 * group_start_code (0x000001B8) marks a new GOP. Scans the bytes already
 * about to be walked for this KLV: mxf_read_packet() has, at the call site,
 * only read the KLV key + BER length, so the AVIO position is exactly this
 * element's payload start - read a small prefix and seek back, leaving the
 * demux cursor exactly where mxf_read_packet() left it.
 *
 * temporal_offset is defined relative to file/storage position (the edit
 * unit that actually holds this picture's data), i.e.
 * temporal_offset = temporal_reference - (edit_unit - gop_start_edit_unit):
 * this is the OPPOSITE axis from the closed file's own IndexTableSegment
 * TemporalOffset field (indexed by display position) - see
 * mxf_growing_set_reordered_pts() for why that matters for first_dts.
 *
 * @return 1 if a picture header was found (temporal_offset/is_key filled),
 *         0 otherwise (both left untouched; edit unit gets temporal_offset 0).
 */
static int mxf_growing_index_mpeg2_temporal_offset(AVFormatContext *s,
                                                    MXFContext *mxf,
                                                    KLVPacket *klv,
                                                    int64_t edit_unit,
                                                    int8_t *temporal_offset,
                                                    int *is_key)
{
    uint8_t buf[MXF_GROWING_MPEG2_SCAN];
    int64_t start = avio_tell(s->pb);
    int64_t want  = FFMIN((int64_t)sizeof(buf), klv->next_klv - start);
    int n, i, found = 0;

    if (want < 8)
        return 0;
    n = avio_read(s->pb, buf, (int)want);
    avio_seek(s->pb, start, SEEK_SET);
    if (n < 8)
        return 0;

    for (i = 0; i + 3 < n; i++) {
        if (buf[i] || buf[i + 1] || buf[i + 2] != 1)
            continue;
        if (buf[i + 3] == 0xb8) {                    /* group_start_code */
            mxf->growing_mpeg2_gop_start_edit_unit = edit_unit;
            mxf->growing_mpeg2_seen_gop = 1;
        } else if (buf[i + 3] == 0x00 && i + 6 <= n) { /* picture_start_code */
            int temporal_reference  = (buf[i + 4] << 2) | (buf[i + 5] >> 6);
            int picture_coding_type = (buf[i + 5] >> 3) & 0x07;

            /* gop_start_edit_unit is only valid once a real group_start_code
             * has been seen since open/takeover/restart - a fresh writer
             * (first-time or promoted from a reader with an empty index) has
             * no way to know where the current GOP actually started, and
             * trusting the zero-initialized/stale default here would clip to
             * a wrong-but-plausible temporal_offset instead of the honest
             * "unknown" of leaving it 0. is_key is unaffected: it comes
             * straight from picture_coding_type, not GOP position. */
            if (mxf->growing_mpeg2_seen_gop) {
                int64_t rel = edit_unit - mxf->growing_mpeg2_gop_start_edit_unit;
                *temporal_offset = av_clip_int8(temporal_reference - (int)rel);
            }
            *is_key = (picture_coding_type == 1); /* I-picture */
            found = 1;
            break; /* one picture header per frame-wrapped essence element */
        }
    }
    return found;
}

/**
 * H.264: reuses the stock parser (av_parser_init(AV_CODEC_ID_H264)) as a
 * black box - precedent for a demuxer feeding it directly outside a decode
 * path already exists in dashenc.c/flacdec.c/demux.c. Its H264ParserContext
 * keeps its own H264POCContext internally (MSB/LSB wraparound state for POC
 * type 0 included) across calls on the same AVCodecParserContext, so
 * output_picture_number/pict_type/key_frame are reliable with no hand-rolled
 * slice-header parsing - confirmed by reading h264_parser.c: it resets that
 * state on every IDR (H264_NAL_IDR_SLICE) and calls the same ff_h264_init_poc()
 * the plan's hand-rolled fallback would have used.
 *
 * MXF's GC AVC essence mapping used in this tree is Annex-B (start-code
 * delimited): mxfdec.c applies no h264_mp4toannexb-style conversion anywhere
 * else for H.264 packets, unlike movdec.c's AVCC essence, so the parser is
 * fed the KLV bytes exactly as demuxed - never touching the packets actually
 * returned to the caller, only this private read-ahead buffer.
 *
 * poc_scale (encoders conventionally step POC by 2 per picture) is measured,
 * not assumed: growing_h264.poc_scale is the running GCD of observed |POC
 * deltas| between consecutive coded pictures in a GOP, converging after a
 * few frames - a deliberately simpler stand-in for
 * mxf_growing_measure_stride()'s fixed 3-sample confirmation, since GCD
 * naturally stabilizes and a wrong transient value only affects rounding of
 * a small offset, never seeking or delivery correctness.
 *
 * @return 1 if the parser produced a usable picture (temporal_offset/is_key
 *         filled), 0 otherwise (e.g. not enough of the slice header was in
 *         the scanned prefix).
 */
static int mxf_growing_index_h264_temporal_offset(AVFormatContext *s,
                                                   MXFContext *mxf,
                                                   KLVPacket *klv,
                                                   AVStream *st,
                                                   int64_t edit_unit,
                                                   int8_t *temporal_offset,
                                                   int *is_key)
{
    uint8_t buf[MXF_GROWING_H264_SCAN];
    int64_t start = avio_tell(s->pb);
    int64_t want  = FFMIN((int64_t)sizeof(buf), klv->next_klv - start);
    const uint8_t *out_data;
    int out_size, n, poc, rel, scale;

    if (want <= 0)
        return 0;

    if (!mxf->growing_h264_parser) {
        mxf->growing_h264_parser = av_parser_init(AV_CODEC_ID_H264);
        if (!mxf->growing_h264_parser)
            return 0;
        mxf->growing_h264_avctx = avcodec_alloc_context3(NULL);
        if (!mxf->growing_h264_avctx ||
            avcodec_parameters_to_context(mxf->growing_h264_avctx, st->codecpar) < 0) {
            av_parser_close(mxf->growing_h264_parser);
            mxf->growing_h264_parser = NULL;
            return 0;
        }
    }

    n = avio_read(s->pb, buf, (int)want);
    avio_seek(s->pb, start, SEEK_SET);
    if (n <= 0)
        return 0;

    av_parser_parse2(mxf->growing_h264_parser, mxf->growing_h264_avctx,
                      (uint8_t **)&out_data, &out_size, buf, n,
                      AV_NOPTS_VALUE, AV_NOPTS_VALUE, klv->offset);

    if (mxf->growing_h264_parser->key_frame) {
        mxf->growing_h264.gop_start_edit_unit = edit_unit;
        mxf->growing_h264.idr_poc  = mxf->growing_h264_parser->output_picture_number;
        mxf->growing_h264.prev_poc = mxf->growing_h264.idr_poc;
        mxf->growing_h264.seen_idr = 1;
        *temporal_offset = 0;
        *is_key = 1;
        return 1;
    }
    if (!mxf->growing_h264.seen_idr)
        return 0; /* no IDR observed yet (e.g. mid-GOP takeover resume): nothing to anchor to */

    poc = mxf->growing_h264_parser->output_picture_number;
    rel = poc - mxf->growing_h264.prev_poc;
    if (rel)
        mxf->growing_h264.poc_scale = mxf->growing_h264.poc_scale
            ? (int)av_gcd(mxf->growing_h264.poc_scale, FFABS(rel)) : FFABS(rel);
    mxf->growing_h264.prev_poc = poc;

    scale = mxf->growing_h264.poc_scale ? mxf->growing_h264.poc_scale : 1;
    *temporal_offset = av_clip_int8((poc - mxf->growing_h264.idr_poc) / scale -
                                     (int)(edit_unit - mxf->growing_h264.gop_start_edit_unit));
    *is_key = 0;
    return 1;
}

static void mxf_growing_index_ref_klv(AVFormatContext *s, KLVPacket *klv,
                                      AVStream *st, MXFTrack *track)
{
    MXFContext *mxf = s->priv_data;
    int32_t size = (int32_t)(klv->next_klv - klv->offset);
    int is_key = track && track->intra_only;
    int8_t temporal_offset = 0;
    uint8_t flags;
    int64_t entry_offset;
    int64_t edit_unit;
    enum AVCodecID codec_id = st ? st->codecpar->codec_id : AV_CODEC_ID_NONE;

    if (mxf->growing_role != MXF_GROWING_ROLE_WRITER)
        return;
    if (!mxf->growing_index_out || mxf->growing_index_disabled)
        return;

    /* OP1a: record the content package's start offset - the system item's,
     * if one was seen since the last content package boundary, else this
     * element's own KLV offset (e.g. OP-Atom, which has no system item) - not
     * this element's own offset unconditionally. A growing OP1a seek must
     * land where ordinary by-key demuxing can walk the whole package. */
    entry_offset = (mxf->growing_cp_start_offset >= 0 &&
                    mxf->growing_cp_start_offset <= klv->offset)
                   ? mxf->growing_cp_start_offset : klv->offset;
    mxf->growing_cp_start_offset = -1;  /* consumed for this content package */

    if (!mxf->growing_index_armed) {
        if (entry_offset < mxf->growing_index_resume_ofs)
            return;                         /* still inside the indexed region */
        if (entry_offset > mxf->growing_index_resume_ofs) {
            av_log(s, AV_LOG_WARNING, "growing MXF: the read started at "
                   "0x%"PRIx64" but the sidecar ends at 0x%"PRIx64"; the index "
                   "would not be dense, so this reader will not extend it\n",
                   entry_offset, mxf->growing_index_resume_ofs);
            mxf_growing_release_sidecar_write(mxf);
            mxf->growing_index_disabled = 1;
            return;
        }
        mxf->growing_index_armed = 1;
        if (mxf->growing_index_entries_written > 0) {
            mxf->growing_index_last_ofs = entry_offset;  /* already on disk */
            return;
        }
        /* an empty sidecar: resume_ofs IS entry 0, so fall through and write it */
    }

    if (entry_offset <= mxf->growing_index_last_ofs)
        return;                             /* re-read after a backward seek */

    /* MPEG-2/H.264 reference tracks: derive temporal_offset (and refine
     * is_key) from the elementary stream itself, never the container's own
     * IndexTableSegment - see mxf_growing_set_reordered_pts() for why. HEVC
     * and other long-GOP codecs get no bitstream-derived reorder here;
     * mxf_growing_available_edit_units() caps their delivery frontier at the
     * container index instead. */
    edit_unit = mxf->growing_index_entries_written;
    if (codec_id == AV_CODEC_ID_MPEG2VIDEO) {
        int mpeg2_key = 0;
        if (mxf_growing_index_mpeg2_temporal_offset(s, mxf, klv, edit_unit,
                                                     &temporal_offset, &mpeg2_key))
            is_key |= mpeg2_key;
    } else if (codec_id == AV_CODEC_ID_H264) {
        int h264_key = 0;
        if (mxf_growing_index_h264_temporal_offset(s, mxf, klv, st, edit_unit,
                                                    &temporal_offset, &h264_key))
            is_key |= h264_key;
    }

    flags = is_key ? MXF_GIDX_FLAG_KEY : 0;

    mxf_growing_write_index_entry(s, entry_offset, size, temporal_offset, flags);
    mxf_growing_vbr_append(mxf, entry_offset, size, temporal_offset, flags);
    mxf->growing_index_last_ofs = entry_offset;
}

/**
 * Refuse a combination the growing/sidecar machinery can never service: a
 * clip-wrapped, audio-only reference stream with no derivable
 * EditUnitByteCount that is not PCM. There is no way to find edit-unit
 * boundaries in essence like that, growing or not - avformat_open_input()
 * must fail cleanly rather than build a sidecar that can never gain an entry.
 *
 * Distinct from the narrower, unconditional (any wrapping/codec) check right
 * after this one in mxf_read_header(): this one exists so the specific,
 * definitely-unfixable combination is refused for a documented reason.
 */
static int mxf_growing_refuse_incompatible_essence(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    AVCodecParameters *par;
    int64_t bits_per_sample;
    int has_video = 0;

    if (!mxf->growing_index_file || !mxf->growing_clip_wrapped ||
        mxf->growing_stride > 0)
        return 0;
    if (mxf->growing_ref_stream < 0 || mxf->growing_ref_stream >= s->nb_streams)
        return 0;

    for (int i = 0; i < s->nb_streams; i++)
        if (s->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
            has_video = 1;
    if (has_video)
        return 0;

    par = s->streams[mxf->growing_ref_stream]->codecpar;
    if (par->codec_type != AVMEDIA_TYPE_AUDIO)
        return 0;

    /* the same PCM test mxf_set_audio_pts() applies, inverted */
    bits_per_sample = par->bits_per_coded_sample;
    if (!bits_per_sample)
        bits_per_sample = av_get_bits_per_sample(par->codec_id);
    if (par->ch_layout.nb_channels > 0 && bits_per_sample > 0)
        return 0;                          /* PCM: already works */

    mxf->growing_refused = 1;
    av_log(s, AV_LOG_ERROR, "growing MXF: refusing -growing_index_file for a "
           "clip-wrapped, non-PCM, audio-only reference stream with no "
           "derivable EditUnitByteCount; no edit-unit boundary can ever be "
           "found\n");
    return 1;
}

/**
 * Undo whatever this process may already have done to growing_index_file
 * before a refusal, so avformat_open_input() failing never leaves a
 * half-created sidecar behind. A reader never created or locked anything, so
 * it has nothing to clean up.
 */
static void mxf_growing_refuse_cleanup(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;

    if (mxf->growing_role == MXF_GROWING_ROLE_WRITER) {
        remove(mxf->growing_index_file);
        mxf_growing_release_sidecar_write(mxf);
    }
    mxf->growing_role = MXF_GROWING_ROLE_NONE;
    mxf->growing      = 0;
}

/**
 * Try to become the writer by locking growing_index_file (creating it if
 * necessary, resuming it if it already exists and loads cleanly); become a
 * reader of it otherwise. Used both for the initial role decision and to
 * recover after the sidecar vanishes mid-run (mxf_growing_reader_wait_step()).
 */
static int mxf_growing_try_claim_sidecar(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int read_ret, resume, ret;

    read_ret = mxf_growing_open_sidecar_read(s);
    /* Only AVERROR_INVALIDDATA means "this is not a valid/resumable sidecar" -
     * safe grounds to recreate (and thereby truncate) the file. Anything else
     * is a transient failure (I/O error, ENOMEM, ...); truncating a file a
     * live writer may still be appending to on the strength of that would be
     * destructive, so fail this open attempt cleanly instead. */
    if (read_ret < 0 && read_ret != AVERROR_INVALIDDATA)
        return read_ret;
    resume = read_ret >= 0;
    /* must run last: releases nothing of ours, and a POSIX record lock dies
     * when this process closes any descriptor to the file - open_sidecar_read
     * has already closed its own transient handle by the time this runs */
    ret = mxf_growing_open_sidecar_write(s, !resume);
    if (ret < 0)
        return ret;

    mxf->growing_role = mxf->growing_index_out ? MXF_GROWING_ROLE_WRITER
                                                : MXF_GROWING_ROLE_READER;
    mxf->growing_cp_start_offset = -1;
    return 0;
}

/**
 * Called from mxf_growing_reader_wait_step() once growing_index_stall_us has
 * elapsed with no index progress. Never itself a give-up condition - a
 * refusal just means the incumbent writer is still alive, and is NOT
 * progress by any definition: it must not touch either progress timestamp.
 *
 * Resume point on success is the index's current frontier (the last entry's
 * offset, or growing_essence_offset if the index is empty), never scratch.
 * Stride/essence-offset are already in mxf-> from this reader's own header-
 * time setup (mxf_growing_assign_role() reads them back rather than
 * measuring, same as any reader) - they are not re-measured here either.
 */
static int mxf_growing_attempt_takeover(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int ret = mxf_growing_try_claim_sidecar(s);

    if (ret < 0)
        return ret;
    if (mxf->growing_role != MXF_GROWING_ROLE_WRITER) {
        av_log(s, AV_LOG_DEBUG, "growing MXF: takeover attempt refused; "
               "still reading\n");
        return 0;
    }

    mxf->growing_index_resume_ofs = mxf->growing_vbr_index->nb_entries
        ? mxf->growing_vbr_index->offsets[mxf->growing_vbr_index->nb_entries - 1]
        : mxf->growing_essence_offset;
    mxf->growing_index_armed       = 0;
    mxf->growing_index_last_ofs    = INT64_MIN;
    mxf->growing_mpeg2_seen_gop    = 0;
    if (avio_seek(s->pb, mxf->growing_index_resume_ofs, SEEK_SET) < 0)
        return AVERROR(EIO);
    mxf->current_klv_data = (KLVPacket){{0}};
    mxf->growing_last_progress_us = av_gettime_relative();
    /* H.264 POC state cannot be reconstructed across the discontinuity - the
     * taking-over writer restarts POC tracking from the first slice header it
     * parses after the resume point (out of scope here; see mxf_growing_h264
     * in MXFContext). MPEG-2's GOP-scoped state has the same problem -
     * growing_mpeg2_seen_gop above makes it wait for a real group_start_code
     * before trusting gop_start_edit_unit, rather than reordering off a stale
     * or zero-initialized value from before the discontinuity. */
    av_log(s, AV_LOG_INFO, "growing MXF: took over as writer at 0x%"PRIx64"\n",
           mxf->growing_index_resume_ofs);
    return 0;
}

/**
 * The sidecar vanished (mxf_growing_reload_sidecar() returned ENOENT): either
 * the writer finished (completion is signaled solely by deletion) or a
 * third party removed it mid-growth. Tell those apart with a footer probe,
 * then either revert to closed or restart as if the index never existed.
 *
 * "Restart" means genuinely from edit unit 0: the essence itself has not
 * changed, only the coordination sidecar, so there is nothing to re-measure -
 * growing_essence_offset from this reader's own header-time setup still
 * applies.
 *
 * @return 0 on success (mxf->growing_role reflects the outcome),
 *         < 0 on I/O failure re-establishing the sidecar or the AVIO position.
 */
static int mxf_growing_reassign_after_sidecar_loss(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int ret;

    ret = mxf_growing_try_claim_sidecar(s);
    if (ret < 0)
        return ret;

    if (mxf->growing_role == MXF_GROWING_ROLE_WRITER) {
        /* the on-disk sidecar was just recreated from scratch (0 entries) -
         * the in-memory index must go back to entry 0 with it, or new entries
         * append on-disk starting at 0 while continuing to append in-memory
         * on top of the stale array. */
        mxf_growing_free_index(mxf);
        mxf->growing_vbr_index = av_mallocz(sizeof(*mxf->growing_vbr_index));
        if (!mxf->growing_vbr_index)
            return AVERROR(ENOMEM);
        mxf->growing_index_resume_ofs = mxf->growing_essence_offset;
        mxf->growing_index_armed      = 0;
        mxf->growing_index_last_ofs   = INT64_MIN;
        mxf->growing_sidecar_duration = 0;
        mxf->growing_mpeg2_seen_gop   = 0;
        if (avio_seek(s->pb, mxf->growing_essence_offset, SEEK_SET) < 0)
            return AVERROR(EIO);
        mxf->current_klv_data = (KLVPacket){{0}};
        mxf->growing_last_progress_us = av_gettime_relative();
        av_log(s, AV_LOG_INFO, "growing MXF: sidecar %s was removed; "
               "restarting it from edit unit 0\n", mxf->growing_index_file);
    } else {
        mxf->growing_last_index_progress_us = av_gettime_relative();
        av_log(s, AV_LOG_INFO, "growing MXF: sidecar %s was removed and "
               "recreated by another process; resuming as a reader\n",
               mxf->growing_index_file);
    }
    mxf->growing_last_takeover_try_us = av_gettime_relative();
    return 0;
}

/**
 * The reader's sibling to mxf_growing_wait_step(): same sticky-state/
 * interrupt/sleep skeleton, but the progress source is sidecar advancement
 * (growing_last_index_progress_us), not essence-stall, and a takeover attempt
 * is layered in on its own, independent cadence (growing_index_stall_us).
 *
 * A reader repeatedly failing to take over an alive-but-slow writer must
 * never time out from that alone - only growing_timeout_us against real
 * index progress (or a directly confirmed footer) ends the wait. The stall
 * interval only decides when a takeover is attempted, never who wins it.
 *
 * @return MXF_WAIT_ROLE_CHANGED if this reader just became the writer (via
 *         takeover or post-loss reassignment) - the AVIO position now sits at
 *         the new resume point, NOT the position this function was called
 *         at, unlike every other return path here.
 */
static int mxf_growing_reader_wait_step(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t saved = avio_tell(s->pb);
    int64_t now, slept;
    int res = MXF_WAIT_RETRY;
    int reload;

    if (mxf->growing_timed_out)
        return MXF_WAIT_TIMEOUT;
    if (mxf->growing_file_closed)
        return MXF_WAIT_FINALIZED;
    if (ff_check_interrupt(&s->interrupt_callback))
        return MXF_WAIT_INTERRUPT;

    now = av_gettime_relative();
    if (mxf->growing_timeout_us > 0 &&
        now - mxf->growing_last_index_progress_us >= mxf->growing_timeout_us) {
        av_log(s, AV_LOG_INFO, "growing MXF: no index progress for %"PRId64
               " us, returning EOF\n", now - mxf->growing_last_index_progress_us);
        mxf->growing_timed_out = 1;
        av_dict_set(&s->metadata, "growing_timed_out", "1", 0);
        return MXF_WAIT_TIMEOUT;
    }

    for (slept = 0; slept < mxf->growing_poll_us;
         slept += MXF_GROWING_SLEEP_SLICE_US) {
        av_usleep(FFMIN(MXF_GROWING_SLEEP_SLICE_US,
                        mxf->growing_poll_us - slept));
        if (ff_check_interrupt(&s->interrupt_callback))
            return MXF_WAIT_INTERRUPT;
    }

    reload = mxf_growing_reload_sidecar(s);
    if (reload == AVERROR(ENOENT)) {
        int fp = mxf_growing_probe_footer(s);

        if (fp > 0) {
            res = MXF_WAIT_FINALIZED;
        } else if (fp == 0) {
            if (mxf_growing_reassign_after_sidecar_loss(s) >= 0 &&
                mxf->growing_role == MXF_GROWING_ROLE_WRITER)
                return MXF_WAIT_ROLE_CHANGED;   /* position is the new resume point */
        }
        /* the index did not advance, it vanished: not progress either way */
    } else if (reload > 0) {
        mxf->growing_last_index_progress_us = now;
    }

    now = av_gettime_relative();
    if (mxf->growing_role == MXF_GROWING_ROLE_READER &&
        now - mxf->growing_last_index_progress_us >= mxf->growing_index_stall_us &&
        now - mxf->growing_last_takeover_try_us >= mxf->growing_index_stall_us) {
        mxf->growing_last_takeover_try_us = now;
        if (mxf_growing_attempt_takeover(s) >= 0 &&
            mxf->growing_role == MXF_GROWING_ROLE_WRITER)
            return MXF_WAIT_ROLE_CHANGED;       /* position is the new resume point */
    }

    if (now - mxf->growing_last_probe_us >= MXF_GROWING_FOOTER_PROBE_US) {
        mxf->growing_last_probe_us = now;
        if (mxf_growing_probe_footer(s) > 0)
            res = MXF_WAIT_FINALIZED;
    }

    if (avio_seek(s->pb, saved, SEEK_SET) < 0) {
        av_log(s, AV_LOG_ERROR, "growing MXF: cannot re-seek to %"PRId64"; the "
               "input does not support range requests\n", saved);
        return MXF_WAIT_ERROR;
    }
    return res;
}

/**
 * Open-time role assignment, replacing the old always-both-roles design.
 *
 * A file that already has a footer gets no role and no sidecar of its own -
 * just a best-effort cleanup of a stale one left by an earlier growing
 * session. A still-growing file with a sidecar requested attempts the write
 * lock: acquired makes this process the writer (which measures the stride
 * and seeds the duration - a reader never does either, it reads them back
 * from the sidecar the writer maintains); refused makes it a reader loading
 * whatever the writer has already published.
 *
 * @param essence_offset the first essence KLV's offset, as found by the
 *                        header parse loop - passed through to
 *                        mxf_growing_measure_stride() when this process
 *                        becomes the writer.
 */
static int mxf_growing_assign_role(AVFormatContext *s, int64_t essence_offset)
{
    MXFContext *mxf = s->priv_data;
    MXFTrack *ref;
    int ret;

    if (!mxf->growing) {
        mxf_growing_cleanup_stale_sidecar(s);
        return 0;
    }

    if (mxf->growing_index_stall_us < mxf->growing_poll_us)
        mxf->growing_index_stall_us = mxf->growing_poll_us;

    mxf_growing_select_ref_stream(s);
    if (mxf->growing_ref_stream < 0) {
        av_log(s, AV_LOG_ERROR, "growing MXF: no usable stream\n");
        return AVERROR_INVALIDDATA;
    }
    if (!mxf->growing_vbr_index) {
        mxf->growing_vbr_index = av_mallocz(sizeof(*mxf->growing_vbr_index));
        if (!mxf->growing_vbr_index)
            return AVERROR(ENOMEM);
    }

    ref = mxf_growing_ref_track(s);
    /* A file still being written carries an Open header partition (ffmpeg's
     * own muxer never closes it until the footer is written, and cannot at
     * all on a non-seekable output), so this is a warning and not a
     * precondition. */
    if (mxf->partitions_count > 0 && !mxf->partitions[0].closed)
        av_log(s, AV_LOG_WARNING, "growing MXF: header partition is open; "
               "using the structural metadata it contains\n");
    mxf->growing_clip_wrapped = ref && ref->wrapping == ClipWrapped;

    /* Decide the role before anything walks the essence forward: a reader
     * must never independently measure the stride or index the reference
     * track - this one change is the fix for the old patch's root defect. */
    ret = mxf_growing_try_claim_sidecar(s);
    if (ret < 0)
        return ret;

    if (mxf->growing_role == MXF_GROWING_ROLE_WRITER) {
        if (!mxf->growing_clip_wrapped) {
            mxf_growing_measure_stride(s, essence_offset);
            /* the sidecar was opened before the stride could be known (a
             * clip-wrapped essence's stride isn't derivable until the essence
             * containers/index tables are computed, later in
             * mxf_read_header()) - patch it now if this measurement found one */
            mxf_growing_patch_sidecar_header(s);
        }
        if (mxf->growing_stride > 0) {
            /* seed the duration now: mxf_compute_index_tables() below reads
             * track->original_duration to fill a zero IndexDuration */
            int64_t dur = mxf_growing_compute_duration(s);
            if (dur != AV_NOPTS_VALUE && dur > 0) {
                for (int i = 0; i < s->nb_streams; i++) {
                    AVStream *st = s->streams[i];
                    MXFTrack *tr = st->priv_data;
                    if (!tr || !tr->edit_rate.num || !ref->edit_rate.num)
                        continue;
                    tr->original_duration = av_rescale_q(dur, av_inv_q(ref->edit_rate),
                                                         av_inv_q(tr->edit_rate));
                    st->duration = av_rescale_q(dur, av_inv_q(ref->edit_rate),
                                                st->time_base);
                }
            }
        }
        mxf->growing_last_progress_us = av_gettime_relative();
    } else {
        mxf->growing_last_index_progress_us = av_gettime_relative();
    }

    mxf->growing_last_takeover_try_us = av_gettime_relative();
    /* back-date so the first wait probes for a footer immediately. Do NOT
     * use INT64_MIN here: the cadence check subtracts this from
     * av_gettime_relative() and would overflow, so the probe would never
     * fire. */
    mxf->growing_last_probe_us = av_gettime_relative() - MXF_GROWING_FOOTER_PROBE_US;
    mxf->growing_last_size     = avio_size(s->pb);
    av_dict_set(&s->metadata, "growing", "1", 0);
    /* observability only, for the multi-process test harness: which role this
     * process ended up with is otherwise only visible via debug logs */
    av_dict_set(&s->metadata, "growing_role",
                mxf->growing_role == MXF_GROWING_ROLE_WRITER ? "writer" : "reader", 0);
    return 0;
}

/* --- growing to closed transition, and the blocking seek ----------------- */

/**
 * Called once a footer partition has been confirmed. Parses the footer's index
 * table segments, rebuilds the index tables and publishes the final duration.
 *
 * The structural metadata is deliberately NOT re-parsed: it would duplicate
 * the streams created from the header partition.
 *
 * NOTE: this frees and rebuilds mxf->index_tables, so it has exactly one call
 * site - the top of mxf_read_packet(), before anything takes a pointer into
 * that array. mxf_read_seek() holds &mxf->index_tables[0] in a local, so
 * transitioning from the shared wait would be a use-after-free.
 */
static int mxf_growing_transition_to_closed(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    int64_t saved = avio_tell(s->pb);
    KLVPacket klv;
    int ret, i;

    /* mxf_read_random_index_pack() resets the position to run_in, so this whole
     * function has to restore it */
    if (!mxf->footer_partition)
        mxf_read_random_index_pack(s);

    if (mxf->footer_partition) {
        ret = avio_seek(s->pb, mxf->run_in + mxf->footer_partition, SEEK_SET);
        if (ret >= 0) {
            /* parse the footer's KLVs, collecting index table segments */
            while (!avio_feof(s->pb)) {
                const MXFMetadataReadTableEntry *metadata;

                ret = klv_read_packet(mxf, &klv, s->pb);
                if (ret < 0)
                    break;
                if (mxf_is_essence_element_key(klv.key) ||
                    IS_KLV_KEY(klv.key, ff_mxf_random_index_pack_key))
                    break;
                if (mxf_is_partition_pack_key(klv.key)) {
                    avio_skip(s->pb, klv.length);
                    continue;
                }
                for (metadata = mxf_metadata_read_table; metadata->read; metadata++) {
                    if (IS_KLV_KEY(klv.key, metadata->key)) {
                        mxf_parse_klv(mxf, klv, metadata->read, metadata->ctx_size,
                                      metadata->type);
                        break;
                    }
                }
                if (!metadata->read)
                    avio_skip(s->pb, klv.length);
            }
        }

        for (i = 0; i < mxf->nb_index_tables; i++) {
            av_freep(&mxf->index_tables[i].segments);
            av_freep(&mxf->index_tables[i].ptses);
            av_freep(&mxf->index_tables[i].fake_index);
            av_freep(&mxf->index_tables[i].offsets);
        }
        av_freep(&mxf->index_tables);
        mxf->nb_index_tables = 0;
        mxf_compute_index_tables(mxf);
    }

    /* publish the final duration */
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MXFTrack *track = st->priv_data;
        MXFIndexTable *t;

        if (!track)
            continue;
        t = mxf_find_index_table(mxf, track->index_sid);
        if (t && t->nb_ptses > 0) {
            st->duration           = t->nb_ptses;
            track->original_duration = t->nb_ptses;
        }
    }

    /* leaving growing mode disables the poll loop; do it before the last
     * refresh so the refresh sees the footer-clamped file size */
    mxf->growing = 0;
    av_dict_set(&s->metadata, "growing", "0", 0);

    /* completion is signaled solely by deleting the sidecar - only the
     * writer that reaches the true end does this; a reader that detects the
     * footer independently (via its own probe, or by walking into the footer
     * partition itself) just reverts, leaving the file for the writer (or a
     * later mxf_growing_cleanup_stale_sidecar() pass) to remove */
    if (mxf->growing_role == MXF_GROWING_ROLE_WRITER && mxf->growing_index_out) {
        remove(mxf->growing_index_file);
        mxf_growing_release_sidecar_write(mxf);
    }
    mxf->growing_role = MXF_GROWING_ROLE_NONE;

    av_log(s, AV_LOG_INFO, "growing MXF: footer reached, switched to closed "
           "mode\n");

    if (avio_seek(s->pb, saved, SEEK_SET) < 0)
        return AVERROR(EIO);
    return 0;
}

/**
 * Block until edit unit sample_time of stream_index is readable.
 *
 * INVARIANT: mutates no demuxer state other than the growing-mode duration and
 * index caches. The AVIO position, mxf->current_klv_data, every
 * track->sample_count and every cur_dts are untouched on every return path, so
 * a subsequent mxf_read_packet() resumes exactly where it was. That is what
 * makes returning AVERROR_EOF from here safe. Do NOT be tempted to clamp
 * sample_time in here.
 *
 * @return 0            the target is available, or availability is unknowable
 *         AVERROR_EOF  the file finalized shorter than the target, or the
 *                      stall timeout expired
 *         AVERROR_EXIT the interrupt callback fired
 */
static int mxf_growing_wait_for_edit_unit(AVFormatContext *s, int stream_index,
                                          int64_t sample_time)
{
    MXFContext *mxf = s->priv_data;
    MXFTrack *src = s->streams[stream_index]->priv_data;
    MXFTrack *ref = mxf_growing_ref_track(s);
    int64_t ref_target, avail, prev_avail = -1;

    if (!mxf->growing || !src || !ref || sample_time <= 0)
        return 0;
    if (!src->edit_rate.num || !ref->edit_rate.num)
        return 0;

    /* availability is counted in reference-track edit units */
    ref_target = av_rescale_q(sample_time, av_inv_q(src->edit_rate),
                              av_inv_q(ref->edit_rate));

    for (;;) {
        avail = mxf_growing_available_edit_units(s);

        if (avail == AV_NOPTS_VALUE) {
            av_log(s, AV_LOG_WARNING, "growing MXF: cannot determine the live "
                   "duration (no stride, no EditUnitByteCount and no sidecar "
                   "entries); not blocking on this seek\n");
            return 0;
        }
        if (ref_target < avail) {
            /* republish before the caller computes an offset against a cached
             * duration */
            mxf_growing_refresh_duration(s);
            return 0;
        }
        if (mxf->growing_file_closed) {
            av_log(s, AV_LOG_ERROR, "growing MXF: seek target edit unit "
                   "%"PRId64" is past the final duration %"PRId64"\n",
                   ref_target, avail);
            return AVERROR_EOF;
        }
        if (avail > prev_avail) {
            prev_avail = avail;
            mxf_growing_note_progress(mxf);
            mxf_growing_refresh_duration(s);
        }

        switch (mxf->growing_role == MXF_GROWING_ROLE_READER
                ? mxf_growing_reader_wait_step(s)
                : mxf_growing_wait_step(s)) {
        case MXF_WAIT_RETRY:
        case MXF_WAIT_FINALIZED:
        case MXF_WAIT_ROLE_CHANGED:
            /* FINALIZED does not mean "stop now": the writer may have appended
             * the last essence AND the footer since our last look, so re-test
             * availability. The growing_file_closed branch above then decides.
             * ROLE_CHANGED (reader took over as writer) is likewise just a
             * reason to re-test: mxf_growing_available_edit_units() is
             * role-aware and picks up the new role on its own. */
            continue;
        case MXF_WAIT_TIMEOUT:
            return AVERROR_EOF;
        case MXF_WAIT_INTERRUPT:
            return AVERROR_EXIT;
        default:
            return AVERROR(EIO);
        }
    }
}

static void mxf_read_random_index_pack(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    uint32_t length;
    int64_t file_size, max_rip_length, min_rip_length;
    KLVPacket klv;

    if (!(s->pb->seekable & AVIO_SEEKABLE_NORMAL))
        return;

    file_size = avio_size(s->pb);

    /* S377m says to check the RIP length for "silly" values, without defining "silly".
     * The limit below assumes a file with nothing but partition packs and a RIP.
     * Before changing this, consider that a muxer may place each sample in its own partition.
     *
     * 105 is the size of the smallest possible PartitionPack
     * 12 is the size of each RIP entry
     * 28 is the size of the RIP header and footer, assuming an 8-byte BER
     */
    max_rip_length = ((file_size - mxf->run_in) / 105) * 12 + 28;
    max_rip_length = FFMIN(max_rip_length, INT_MAX); //2 GiB and up is also silly

    /* We're only interested in RIPs with at least two entries.. */
    min_rip_length = 16+1+24+4;

    /* See S377m section 11 */
    avio_seek(s->pb, file_size - 4, SEEK_SET);
    length = avio_rb32(s->pb);

    if (length < min_rip_length || length > max_rip_length)
        goto end;
    avio_seek(s->pb, file_size - length, SEEK_SET);
    if (klv_read_packet(mxf, &klv, s->pb) < 0 ||
        !IS_KLV_KEY(klv.key, ff_mxf_random_index_pack_key))
        goto end;
    if (klv.next_klv != file_size || klv.length <= 4 || (klv.length - 4) % 12) {
        av_log(s, AV_LOG_WARNING, "Invalid RIP KLV length\n");
        goto end;
    }

    avio_skip(s->pb, klv.length - 12);
    mxf->footer_partition = avio_rb64(s->pb);

    /* sanity check */
    if (mxf->run_in + mxf->footer_partition >= file_size) {
        av_log(s, AV_LOG_WARNING, "bad FooterPartition in RIP - ignoring\n");
        mxf->footer_partition = 0;
    }

end:
    avio_seek(s->pb, mxf->run_in, SEEK_SET);
}

static int mxf_read_header(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;
    KLVPacket klv;
    int64_t essence_offset = 0;
    int ret;
    int64_t run_in;

    mxf->last_forward_tell = INT64_MAX;

    if (!mxf_read_sync(s->pb, mxf_header_partition_pack_key, 14)) {
        av_log(s, AV_LOG_ERROR, "could not find header partition pack key\n");
        return AVERROR_INVALIDDATA;
    }
    avio_seek(s->pb, -14, SEEK_CUR);
    mxf->fc = s;
    run_in = avio_tell(s->pb);
    if (run_in < 0 || run_in > RUN_IN_MAX)
        return AVERROR_INVALIDDATA;
    mxf->run_in = run_in;

    /* Always read the RIP, even in growing mode: if the writer finished before
     * we opened the file we want the real footer index rather than a stride
     * estimate. Safe on a genuinely growing file - the RIP check validates
     * klv.next_klv == file_size, so trailing essence cannot be mistaken for a
     * RIP, and it leaves the position at mxf->run_in either way. */
    mxf_read_random_index_pack(s);

    /* Mode is decided solely by growing_index_file's presence plus
     * footer-absence - there is no separate user flag any more. The final
     * decision cannot be made yet: some multi-partition MXF files are legally
     * closed with no trailing RandomIndexPack, and footer-partition presence
     * may only become known later in the partition walk below (a body
     * partition's own PartitionPack can carry FooterPartition). Deciding here
     * would latch a stale "growing" verdict and break header parsing of such
     * a closed file at its first essence KLV. */

    while (!avio_feof(s->pb)) {
        size_t x;

        ret = klv_read_packet(mxf, &klv, s->pb);
        if (ret < 0 || IS_KLV_KEY(klv.key, ff_mxf_random_index_pack_key)) {
            if (ret >= 0 && avio_size(s->pb) > klv.next_klv)
                av_log(s, AV_LOG_WARNING, "data after the RandomIndexPack, assuming end of file\n");
            /* EOF - seek to previous partition or stop */
            if(mxf_parse_handle_partition_or_eof(mxf) <= 0)
                break;
            else
                continue;
        }

        PRINT_KEY(s, "read header", klv.key);
        av_log(s, AV_LOG_TRACE, "size %"PRIu64" offset %#"PRIx64"\n", klv.length, klv.offset);
        if (mxf_match_uid(klv.key, mxf_encrypted_triplet_key, sizeof(mxf_encrypted_triplet_key)) ||
            IS_KLV_KEY(klv.key, mxf_essence_element_key) ||
            IS_KLV_KEY(klv.key, mxf_canopus_essence_element_key) ||
            IS_KLV_KEY(klv.key, mxf_avid_essence_element_key) ||
            IS_KLV_KEY(klv.key, mxf_system_item_key_cp) ||
            IS_KLV_KEY(klv.key, mxf_system_item_key_gc)) {

            if (!mxf->current_partition) {
                av_log(mxf->fc, AV_LOG_ERROR, "found essence prior to first PartitionPack\n");
                return AVERROR_INVALIDDATA;
            }

            if (!mxf->current_partition->first_essence_klv.offset)
                mxf->current_partition->first_essence_klv = klv;

            if (!essence_offset)
                essence_offset = klv.offset;

            /* Final growing/non-growing decision: test the current (possibly
             * just-updated-by-this-walk) footer_partition rather than a value
             * latched before the walk started. If still unknown, this is a
             * growing file (or one about to be treated as such) - stop header
             * parsing as soon as we find essence. */
            if (mxf->growing_index_file && !mxf->footer_partition) {
                mxf->growing = 1;
                break;
            }

            /* seek to footer, previous partition or stop */
            if (mxf_parse_handle_essence(mxf) <= 0)
                break;
            continue;
        } else if (mxf_is_partition_pack_key(klv.key) && mxf->current_partition) {
            /* next partition pack - keep going, seek to previous partition or stop */
            if(mxf_parse_handle_partition_or_eof(mxf) <= 0)
                break;
            else if (mxf->parsing_backward)
                continue;
            /* we're still parsing forward. proceed to parsing this partition pack */
        }

        for (x = 0; x < FF_ARRAY_ELEMS(mxf_metadata_read_table); x++) {
            const MXFMetadataReadTableEntry *metadata = &mxf_metadata_read_table[x];
            if (IS_KLV_KEY(klv.key, metadata->key)) {
                if (metadata->read) {
                    if ((ret = mxf_parse_klv(mxf, klv, metadata->read, metadata->ctx_size, metadata->type)) < 0)
                        return ret;
                } else {
                    avio_skip(s->pb, klv.length);
                }
                break;
            }
        }
        if (x >= FF_ARRAY_ELEMS(mxf_metadata_read_table)) {
            av_log(s, AV_LOG_VERBOSE, "Dark key " PRIxUID "\n",
                            UID_ARG(klv.key));
            avio_skip(s->pb, klv.length);
        }
    }

    if (mxf->growing_index_file && !mxf->growing) {
        av_log(s, AV_LOG_INFO, "growing MXF: a footer is already present, "
               "opening as a closed file\n");
        av_dict_set(&s->metadata, "growing", "0", 0);
    }

    /* FIXME avoid seek */
    if (!essence_offset)  {
        av_log(s, AV_LOG_ERROR, "no essence\n");
        return AVERROR_INVALIDDATA;
    }
    avio_seek(s->pb, essence_offset, SEEK_SET);

    /* we need to do this before computing the index tables
     * to be able to fill in zero IndexDurations with st->duration */
    if ((ret = mxf_parse_structural_metadata(mxf)) < 0)
        return ret;

    /* ---- growing phase A: everything mxf_handle_missing_index_segment() and
     * mxf_compute_index_tables() depend on. They consume st->duration and
     * track->original_duration to fill a zero IndexDuration, so the role
     * (which decides whether this process may measure the stride at all) and
     * the seed duration have to be established first. ---- */
    if (mxf->growing_index_file) {
        ret = mxf_growing_assign_role(s, essence_offset);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < s->nb_streams; i++)
        mxf_handle_missing_index_segment(mxf, s->streams[i]);

    if ((ret = mxf_compute_index_tables(mxf)) < 0)
        return ret;

    if (mxf->nb_index_tables > 1) {
        /* TODO: look up which IndexSID to use via EssenceContainerData */
        av_log(mxf->fc, AV_LOG_INFO, "got %i index tables - only the first one (IndexSID %i) will be used\n",
               mxf->nb_index_tables, mxf->index_tables[0].index_sid);
    } else if (mxf->nb_index_tables == 0 && mxf->op == OPAtom && (s->error_recognition & AV_EF_EXPLODE)) {
        av_log(mxf->fc, AV_LOG_ERROR, "cannot demux OPAtom without an index\n");
        return AVERROR_INVALIDDATA;
    }

    mxf_compute_essence_containers(s);

    for (int i = 0; i < s->nb_streams; i++)
        mxf_compute_edit_units_per_packet(mxf, s->streams[i]);

    /* ---- growing phase B: needs p->essence_offset from
     * mxf_compute_essence_containers() and, for clip-wrapped essence, the
     * EditUnitByteCount that mxf_handle_missing_index_segment() or a real
     * IndexTableSegment supplies. Only reached with an assigned role - a
     * closed file with growing_index_file set got only a stale-sidecar
     * cleanup in mxf_growing_assign_role() and has nothing more to do. ---- */
    if (mxf->growing_role != MXF_GROWING_ROLE_NONE) {
        MXFTrack *ref = mxf_growing_ref_track(s);

        for (int i = 0; ref && i < mxf->partitions_count; i++) {
            if (mxf->partitions[i].body_sid == ref->body_sid &&
                mxf->partitions[i].essence_offset > 0) {
                if (mxf->growing_clip_wrapped)
                    mxf->growing_essence_offset = mxf->partitions[i].essence_offset;
                break;
            }
        }

        if (mxf->growing_clip_wrapped && ref) {
            MXFIndexTable *t = mxf_find_index_table(mxf, ref->index_sid);
            if (t && t->nb_segments > 0 && t->segments[0]->edit_unit_byte_count > 0) {
                /* one formula covers both wrappings: with elem_size == stride,
                 * the duration expression collapses to avail / eubc */
                mxf->growing_stride    = t->segments[0]->edit_unit_byte_count;
                mxf->growing_elem_size = mxf->growing_stride;
            }
            mxf->growing_stride_done = 1;
            /* the sidecar was opened in mxf_growing_assign_role(), before a
             * clip-wrapped stride could be known - patch it in now */
            if (mxf->growing_role == MXF_GROWING_ROLE_WRITER)
                mxf_growing_patch_sidecar_header(s);
        }

        /* The sidecar is a consumed interface: if it was asked for, it must
         * exist and be maintained. Refuse rather than create one that can
         * never gain an entry - cleaning up anything this process may already
         * have created so avformat_open_input() failing leaves nothing behind. */
        if (mxf_growing_refuse_incompatible_essence(s)) {
            mxf_growing_refuse_cleanup(s);
            return AVERROR_INVALIDDATA;
        }
        if (mxf->growing_stride <= 0 && mxf->growing_clip_wrapped) {
            av_log(s, AV_LOG_ERROR, "growing MXF: cannot index clip-wrapped "
                   "essence with no IndexTableSegment and no derivable "
                   "EditUnitByteCount; retry without -growing_index_file\n");
            mxf_growing_refuse_cleanup(s);
            return AVERROR_INVALIDDATA;
        }

        if (mxf->growing_role == MXF_GROWING_ROLE_WRITER) {
            /* For a brand-new sidecar, resume at essence_offset (the local
             * var from the header parse loop) rather than
             * mxf->growing_essence_offset. They differ by the leading
             * system item's size in OP1a: growing_essence_offset is
             * deliberately the reference track's own first essence element
             * (mxf_growing_measure_stride()/mxf_growing_observe_klv() only
             * ever record that, to keep the CBR stride/duration arithmetic
             * video-to-video), but demuxing below resumes at essence_offset,
             * the content package's true start - the same offset
             * mxf_growing_index_ref_klv()'s OP1a fix records for entry 0 via
             * growing_cp_start_offset. Seeding resume_ofs from
             * growing_essence_offset instead made entry 0's (correctly
             * earlier) offset look like it was already indexed - silently
             * skipped without arming - so the dense-gap guard then saw entry
             * 1 as a permanent gap and gave up on indexing for good. */
            mxf->growing_index_resume_ofs = mxf->growing_vbr_index->nb_entries
                ? mxf->growing_vbr_index->offsets[mxf->growing_vbr_index->nb_entries - 1]
                : essence_offset;
            mxf->growing_index_armed    = 0;
            mxf->growing_index_last_ofs = INT64_MIN;
        }

        if (mxf->growing)
            mxf_growing_refresh_duration(s);

        /* mxf_growing_measure_stride() and the sidecar work restore the AVIO
         * position, but be explicit: demuxing starts at the first essence KLV */
        avio_seek(s->pb, essence_offset, SEEK_SET);
    }

    return 0;
}

/* Get the edit unit of the next packet from current_offset in a track. The returned edit unit can be original_duration as well! */
static int mxf_get_next_track_edit_unit(MXFContext *mxf, MXFTrack *track, int64_t current_offset, int64_t *edit_unit_out)
{
    int64_t a, b, m, offset;
    MXFIndexTable *t = mxf_find_index_table(mxf, track->index_sid);

    /* in growing mode the binary search bound, the index segment duration and
     * the partition essence length are all latched at open time; refresh them
     * together so they cannot drift apart */
    if (mxf->growing)
        mxf_growing_refresh_duration(mxf->fc);

    if (!t || track->original_duration <= 0)
        return -1;

    a = -1;
    b = track->original_duration;
    while (b - 1 > a) {
        m = (a + (uint64_t)b) >> 1;
        if (mxf_edit_unit_absolute_offset(mxf, t, m, track->edit_rate, NULL, &offset, NULL, 0) < 0)
            return -1;
        if (offset < current_offset)
            a = m;
        else
            b = m;
    }

    *edit_unit_out = b;

    return 0;
}

static int64_t mxf_compute_sample_count(MXFContext *mxf, AVStream *st,
                                        int64_t edit_unit)
{
    MXFTrack *track = st->priv_data;
    AVRational time_base = av_inv_q(track->edit_rate);
    AVRational sample_rate = av_inv_q(st->time_base);

    // For non-audio sample_count equals current edit unit
    if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO)
        return edit_unit;

    if ((sample_rate.num / sample_rate.den) == 48000) {
        return av_rescale_q(edit_unit, sample_rate, track->edit_rate);
    } else {
        int64_t remainder = (sample_rate.num * (int64_t)  time_base.num) %
                            (  time_base.den * (int64_t)sample_rate.den);
        if (remainder)
            av_log(mxf->fc, AV_LOG_WARNING,
                   "seeking detected on stream #%d with time base (%d/%d) and "
                   "sample rate (%d/%d), audio pts won't be accurate.\n",
                   st->index, time_base.num, time_base.den,
                   sample_rate.num, sample_rate.den);
        return av_rescale_q(edit_unit, sample_rate, track->edit_rate);
    }
}

/**
 * Make sure track->sample_count is correct based on what offset we're currently at.
 * Also determine the next edit unit (or packet) offset.
 * @return next_ofs if OK, <0 on error
 */
static int64_t mxf_set_current_edit_unit(MXFContext *mxf, AVStream *st, int64_t current_offset, int resync)
{
    int64_t next_ofs = -1;
    MXFTrack *track = st->priv_data;
    int64_t edit_unit = av_rescale_q(track->sample_count, st->time_base, av_inv_q(track->edit_rate));
    int64_t new_edit_unit;
    MXFIndexTable *t = mxf_find_index_table(mxf, track->index_sid);

    if (!t || track->wrapping == UnknownWrapped || edit_unit > INT64_MAX - track->edit_units_per_packet)
        return -1;

    if (mxf_edit_unit_absolute_offset(mxf, t, edit_unit + track->edit_units_per_packet, track->edit_rate, NULL, &next_ofs, NULL, 0) < 0 &&
        (next_ofs = mxf_essence_container_end(mxf, t->body_sid)) <= 0) {
        av_log(mxf->fc, AV_LOG_ERROR, "unable to compute the size of the last packet\n");
        return -1;
    }

    /* check if the next edit unit offset (next_ofs) starts ahead of current_offset */
    if (next_ofs > current_offset)
        return next_ofs;

    if (!resync) {
        av_log(mxf->fc, AV_LOG_ERROR, "cannot find current edit unit for stream %d, invalid index?\n", st->index);
        return -1;
    }

    if (mxf_get_next_track_edit_unit(mxf, track, current_offset + 1, &new_edit_unit) < 0 || new_edit_unit <= 0) {
        av_log(mxf->fc, AV_LOG_ERROR, "failed to find next track edit unit in stream %d\n", st->index);
        return -1;
    }

    new_edit_unit--;
    track->sample_count = mxf_compute_sample_count(mxf, st, new_edit_unit);
    av_log(mxf->fc, AV_LOG_WARNING, "edit unit sync lost on stream %d, jumping from %"PRId64" to %"PRId64"\n", st->index, edit_unit, new_edit_unit);

    return mxf_set_current_edit_unit(mxf, st, current_offset, 0);
}

static int mxf_set_audio_pts(MXFContext *mxf, AVCodecParameters *par,
                             AVPacket *pkt)
{
    AVStream *st = mxf->fc->streams[pkt->stream_index];
    MXFTrack *track = st->priv_data;
    int64_t bits_per_sample = par->bits_per_coded_sample;

    if (!bits_per_sample)
        bits_per_sample = av_get_bits_per_sample(par->codec_id);

    pkt->pts = track->sample_count;

    if (par->ch_layout.nb_channels <= 0 ||
        bits_per_sample <= 0            ||
        par->ch_layout.nb_channels * (int64_t)bits_per_sample < 8)
        track->sample_count = mxf_compute_sample_count(mxf, st, av_rescale_q(track->sample_count, st->time_base, av_inv_q(track->edit_rate)) + 1);
    else
        track->sample_count += pkt->size / (par->ch_layout.nb_channels * (int64_t)bits_per_sample / 8);

    return 0;
}

/**
 * PTS/DTS for a growing reference-track packet, derived from the dense
 * sidecar's per-edit-unit temporal_offset (mxf_growing_index_ref_klv()'s
 * MPEG-2/H.264 helpers, for writer or reader alike - a reader's reloaded
 * mirror has the same entries).
 *
 * This reuses mxf_compute_ptses_fake_index()'s DTS convention
 * (DTS = edit_unit + first_dts, keeping DTS <= PTS) but computes PTS
 * directly as edit_unit + temporal_offset[edit_unit], not via that
 * function's scatter/bucket-sort into a ptses[] array: our temporal_offset
 * is indexed by, and defined relative to, FILE/storage position (the edit
 * unit that physically holds this packet's data) - the OPPOSITE axis from
 * the closed file's own IndexTableSegment TemporalOffset field, which is
 * indexed by DISPLAY position. On that axis the direct formula is exact
 * (verified against mxf_compute_ptses_fake_index()'s own worked example in
 * its comment, translating it onto this axis), so first_dts here is the
 * running MINIMUM of temporal_offset seen so far - not the negated maximum
 * that function uses, which is the closed-file convention's own axis, not
 * ours. Stabilizes once the first GOP's offsets have all been observed.
 *
 * Leaves pkt->pts/dts untouched (AV_NOPTS_VALUE) if the sidecar does not yet
 * reach this edit unit - should not happen in practice, since both the
 * writer (indexes this edit unit earlier in the same mxf_read_packet() KLV
 * walk) and the reader (blocked on the index frontier before delivering
 * this far) already guarantee it, but this must never invent a DTS/PTS.
 */
static void mxf_growing_set_reordered_pts(MXFContext *mxf, AVPacket *pkt,
                                          int64_t edit_unit)
{
    MXFGrowingIndex *gi = mxf->growing_vbr_index;
    int8_t off;

    if (!gi || edit_unit < 0 || edit_unit >= gi->nb_entries)
        return;

    off = gi->temporal_offsets[edit_unit];
    if (off < gi->min_temporal_offset)
        gi->min_temporal_offset = off;

    pkt->dts = edit_unit + gi->min_temporal_offset;
    pkt->pts = edit_unit + off;
}

static int mxf_set_pts(MXFContext *mxf, AVStream *st, AVPacket *pkt)
{
    AVCodecParameters *par = st->codecpar;
    MXFTrack *track = st->priv_data;

    if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
        /* Growing MPEG-2/H.264 reference track: this MUST take priority over
         * the container-index branch below. That branch derives PTS/DTS from
         * the container's own IndexTableSegment, which a live writer
         * publishes only every EDIT_UNITS_PER_BODY edit units (mxfenc.c) -
         * up to ~10s behind the write frontier at 25fps. Relying on it here
         * is exactly the defect this subsystem exists to fix (~40% of
         * B-frame packets coming back with pts == AV_NOPTS_VALUE). Excludes
         * intra-only explicitly: there is nothing to reorder, and that
         * branch already handles it correctly (PTS = EditUnit, DTS left for
         * utils.c to derive). Codec-gated to MPEG-2/H.264: those are the only
         * codecs mxf_growing_index_ref_klv() derives temporal_offset for
         * (see mxf_growing_available_edit_units()) - HEVC/other long-GOP
         * codecs get temporal_offset == 0 for every entry, which would
         * silently produce pts == dts here instead of falling through to the
         * container-index branch that the availability cap already
         * guarantees is populated for them. */
        if (mxf->growing_role != MXF_GROWING_ROLE_NONE &&
            st->index == mxf->growing_ref_stream && mxf->growing_vbr_index &&
            !track->intra_only &&
            (par->codec_id == AV_CODEC_ID_MPEG2VIDEO ||
             par->codec_id == AV_CODEC_ID_H264)) {
            mxf_growing_set_reordered_pts(mxf, pkt, track->sample_count);
        } else {
            /* see if we have an index table to derive timestamps from */
            MXFIndexTable *t = mxf_find_index_table(mxf, track->index_sid);

            if (t && track->sample_count < t->nb_ptses) {
                pkt->dts = track->sample_count + t->first_dts;
                pkt->pts = t->ptses[track->sample_count];
            } else if (track->intra_only) {
                /* intra-only -> PTS = EditUnit.
                 * let utils.c figure out DTS since it can be < PTS if low_delay = 0 (Sony IMX30) */
                pkt->pts = track->sample_count;
            }
        }
        track->sample_count++;
    } else if (par->codec_type == AVMEDIA_TYPE_AUDIO) {
        int ret = mxf_set_audio_pts(mxf, par, pkt);
        if (ret < 0)
            return ret;
    } else if (track) {
        pkt->dts = pkt->pts = track->sample_count;
        pkt->duration = 1;
        track->sample_count++;
    }
    return 0;
}

static int mxf_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    KLVPacket klv;
    MXFContext *mxf = s->priv_data;
    int ret;

    /* The one and only call site for the transition: it frees and rebuilds
     * mxf->index_tables, and nothing below holds a pointer into that array
     * yet. mxf_read_seek() keeps &mxf->index_tables[0] in a local, so
     * transitioning from the shared wait would be a use-after-free. */
    if (mxf->growing && mxf->growing_file_closed) {
        if ((ret = mxf_growing_transition_to_closed(s)) < 0)
            return ret;
    }

    while (1) {
        int64_t max_data_size;
        int64_t pos = avio_tell(s->pb);

        if (pos < mxf->current_klv_data.next_klv - mxf->current_klv_data.length || pos >= mxf->current_klv_data.next_klv) {
            mxf->current_klv_data = (KLVPacket){{0}};
            if (mxf->growing) {
                /* Growing read: retry across both end-of-data and a KLV the
                 * writer has not finished, waiting for the file to grow. */
                int64_t resume_pos = avio_tell(s->pb);
                int last_try = 0;

                for (;;) {
                    ret = klv_read_packet(mxf, &klv, s->pb);
                    if (ret >= 0)
                        break;
                    if (!avio_feof(s->pb))
                        break;                  /* a real error */
                    if (last_try)
                        break;                  /* finalized and still short */

                    /* klv_read_packet() may have consumed bytes while syncing */
                    avio_seek(s->pb, resume_pos, SEEK_SET);

                    if (mxf->growing_role == MXF_GROWING_ROLE_READER) {
                        switch (mxf_growing_reader_wait_step(s)) {
                        case MXF_WAIT_RETRY:
                            break;
                        case MXF_WAIT_ROLE_CHANGED:
                            /* this reader just became the writer: resume from
                             * the sidecar's frontier, not our old blocked
                             * reader position - the AVIO position is already
                             * there */
                            resume_pos = avio_tell(s->pb);
                            break;
                        case MXF_WAIT_FINALIZED:
                            last_try = 1;
                            break;
                        case MXF_WAIT_TIMEOUT:
                            return AVERROR_EOF;
                        case MXF_WAIT_INTERRUPT:
                            return AVERROR_EXIT;
                        default:
                            return AVERROR(EIO);
                        }
                    } else {
                        switch (mxf_growing_wait_step(s)) {
                        case MXF_WAIT_RETRY:
                            break;
                        case MXF_WAIT_FINALIZED:
                            /* the writer may have appended the final essence KLV
                             * AND the footer since our last look, so make one more
                             * attempt before giving up */
                            last_try = 1;
                            break;
                        case MXF_WAIT_TIMEOUT:
                            return AVERROR_EOF;
                        case MXF_WAIT_INTERRUPT:
                            return AVERROR_EXIT;
                        default:
                            return AVERROR(EIO);
                        }
                    }
                }
                if (ret < 0) {
                    if (mxf->growing_file_closed) {
                        mxf_growing_transition_to_closed(s);
                        return AVERROR_EOF;
                    }
                    break;
                }
                mxf_growing_refresh_duration(s);
            } else {
                ret = klv_read_packet(mxf, &klv, s->pb);
                if (ret < 0)
                    break;
            }

            /* Deterministic finalization: the footer partition pack is a KLV
             * like any other, so detect it from a key we actually read rather
             * than peeking at the end of the file. Falls through to skip:
             * below, which drains the footer's own KLVs. */
            if (mxf->growing && mxf_is_partition_pack_key(klv.key) &&
                klv.key[13] == 4) {
                mxf->growing_file_closed   = 1;
                mxf->growing_footer_offset = klv.offset - mxf->run_in;
                if (!mxf->footer_partition)
                    mxf->footer_partition = mxf->growing_footer_offset;
                av_log(s, AV_LOG_INFO, "growing MXF: footer partition reached "
                       "at 0x%"PRIx64"\n", klv.offset);
            }
            /* OP1a offset fix (see mxf_growing_index_ref_klv()): remember the
             * earliest KLV offset since the last content package boundary,
             * so the entry eventually recorded for the reference track's
             * essence element points at the content package's start, not the
             * element's own offset. Writer-only: a reader never indexes. */
            if (mxf->growing_role == MXF_GROWING_ROLE_WRITER &&
                mxf->growing_cp_start_offset < 0 &&
                (IS_KLV_KEY(klv.key, mxf_system_item_key_cp) ||
                 IS_KLV_KEY(klv.key, mxf_system_item_key_gc)))
                mxf->growing_cp_start_offset = klv.offset;
            // klv.key[0..3] == mxf_klv_key from here forward
            max_data_size = klv.length;
            pos = klv.next_klv - klv.length;
            PRINT_KEY(s, "read packet", klv.key);
            av_log(s, AV_LOG_TRACE, "size %"PRIu64" offset %#"PRIx64"\n", klv.length, klv.offset);
            if (mxf_match_uid(klv.key, mxf_encrypted_triplet_key, sizeof(mxf_encrypted_triplet_key))) {
                ret = mxf_decrypt_triplet(s, pkt, &klv);
                if (ret < 0) {
                    av_log(s, AV_LOG_ERROR, "invalid encoded triplet\n");
                    return ret;
                }
                return 0;
            }
        } else {
            klv = mxf->current_klv_data;
            max_data_size = klv.next_klv - pos;
        }
        if (mxf_match_uid(klv.key, mxf_essence_element_key, 12) ||
            IS_KLV_KEY(klv.key, mxf_canopus_essence_element_key) ||
            IS_KLV_KEY(klv.key, mxf_avid_essence_element_key)) {
            int body_sid = find_body_sid_by_absolute_offset(mxf, klv.offset);
            int index = mxf_get_stream_index(s, &klv, body_sid);
            int64_t next_ofs;
            AVStream *st;
            MXFTrack *track;

            if (index < 0) {
                av_log(s, AV_LOG_ERROR,
                       "error getting stream index %"PRIu32"\n",
                       AV_RB32(klv.key + 12));
                goto skip;
            }

            st = s->streams[index];
            track = st->priv_data;

            /* A frame-wrapped KLV is returned whole, so it must be fully
             * written; klv.next_klv is the real packet end here. */
            if (mxf->growing && track && track->wrapping == FrameWrapped &&
                mxf_growing_incomplete(s, klv.next_klv)) {
                int act = mxf_growing_wait_for_data(s, klv.offset);

                if (act < 0)
                    return act;
                if (act == 0) {
                    /* finalized while short: this edit unit never completes */
                    mxf_growing_transition_to_closed(s);
                    return AVERROR_EOF;
                }
                mxf->current_klv_data = (KLVPacket){{0}};
                continue;
            }

            /* Index and observe BEFORE the discard check: with -map 0:a on a
             * video-reference file the reference stream is discarded, and
             * skipping this would silently stop extending the sidecar.
             * Writer-only: a reader trusts the sidecar the writer maintains
             * and never independently measures or indexes anything itself -
             * this is the fix for the old patch's root defect. */
            if (mxf->growing && mxf->growing_role == MXF_GROWING_ROLE_WRITER) {
                mxf_growing_observe_klv(s, &klv, index);
                if (index == mxf->growing_ref_stream && !mxf->growing_clip_wrapped)
                    mxf_growing_index_ref_klv(s, &klv, st, track);
            }

            if (s->streams[index]->discard == AVDISCARD_ALL)
                goto skip;

            next_ofs = mxf_set_current_edit_unit(mxf, st, pos, 1);

            if (track->wrapping != FrameWrapped) {
                int64_t size;

                if (next_ofs <= 0) {
                    // If we have no way to packetize the data, then return it in chunks...
                    if (klv.next_klv - klv.length == pos && max_data_size > MXF_MAX_CHUNK_SIZE) {
                        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
                        avpriv_request_sample(s, "Huge KLV without proper index in non-frame wrapped essence");
                    }
                    size = FFMIN(max_data_size, MXF_MAX_CHUNK_SIZE);
                } else {
                    if ((size = next_ofs - pos) <= 0) {
                        av_log(s, AV_LOG_ERROR, "bad size: %"PRId64"\n", size);
                        mxf->current_klv_data = (KLVPacket){{0}};
                        return AVERROR_INVALIDDATA;
                    }
                    // We must not overread, because the next edit unit might be in another KLV
                    if (size > max_data_size)
                        size = max_data_size;
                }

                mxf->current_klv_data = klv;
                klv.offset = pos;
                klv.length = size;
                klv.next_klv = klv.offset + klv.length;

                /* Clip-wrapped: only this edit-unit sub-range has to be
                 * present, not the whole KLV. current_klv_data is left set so
                 * the retry re-enters through the reuse branch at the top of
                 * the loop instead of trying to re-read a key mid-essence. */
                if (mxf->growing && mxf_growing_incomplete(s, klv.next_klv)) {
                    int act = mxf_growing_wait_for_data(s, pos);

                    if (act < 0)
                        return act;
                    if (act == 0) {
                        mxf_growing_transition_to_closed(s);
                        return AVERROR_EOF;
                    }
                    continue;
                }
            }

            /* check for 8 channels AES3 element */
            if (klv.key[12] == 0x06 && klv.key[13] == 0x01 && klv.key[14] == 0x10) {
                ret = mxf_get_d10_aes3_packet(s->pb, s->streams[index],
                                              pkt, klv.length);
                if (ret < 0) {
                    av_log(s, AV_LOG_ERROR, "error reading D-10 aes3 frame\n");
                    mxf->current_klv_data = (KLVPacket){{0}};
                    return ret;
                }
            } else if (mxf->eia608_extract &&
                       s->streams[index]->codecpar->codec_id == AV_CODEC_ID_EIA_608) {
                ret = mxf_get_eia608_packet(s, s->streams[index], pkt, klv.length);
                if (ret < 0) {
                    mxf->current_klv_data = (KLVPacket){{0}};
                    return ret;
                }
            } else {
                ret = av_get_packet(s->pb, pkt, klv.length);
                if (ret < 0) {
                    mxf->current_klv_data = (KLVPacket){{0}};
                    return ret;
                }
            }
            pkt->stream_index = index;
            pkt->pos = klv.offset;

            ret = mxf_set_pts(mxf, st, pkt);
            if (ret < 0) {
                mxf->current_klv_data = (KLVPacket){{0}};
                return ret;
            }

            /* seek for truncated packets */
            avio_seek(s->pb, klv.next_klv, SEEK_SET);

            /* Delivering a packet is the definition of progress for the stall
             * timer. Note it here rather than on a successful klv_read_packet():
             * an incomplete KLV is re-read on every retry, and counting that as
             * progress reset the timer forever. */
            if (mxf->growing)
                mxf_growing_note_progress(mxf);

            return 0;
        } else {
        skip:
            avio_skip(s->pb, max_data_size);
            mxf->current_klv_data = (KLVPacket){{0}};
        }
    }
    return avio_feof(s->pb) ? AVERROR_EOF : ret;
}

static int mxf_read_close(AVFormatContext *s)
{
    MXFContext *mxf = s->priv_data;

    av_freep(&mxf->packages_refs);
    av_freep(&mxf->essence_container_data_refs);

    for (int i = 0; i < s->nb_streams; i++)
        s->streams[i]->priv_data = NULL;

    for (int type = 0; type < FF_ARRAY_ELEMS(mxf->metadata_set_groups); type++) {
        MXFMetadataSetGroup *mg = &mxf->metadata_set_groups[type];
        for (int i = 0; i < mg->metadata_sets_count; i++)
            mxf_free_metadataset(mg->metadata_sets + i, type);
        mg->metadata_sets_count = 0;
        av_freep(&mg->metadata_sets);
    }
    av_freep(&mxf->partitions);
    av_freep(&mxf->aesc);
    av_freep(&mxf->local_tags);

    if (mxf->index_tables) {
        for (int i = 0; i < mxf->nb_index_tables; i++) {
            av_freep(&mxf->index_tables[i].segments);
            av_freep(&mxf->index_tables[i].ptses);
            av_freep(&mxf->index_tables[i].fake_index);
            av_freep(&mxf->index_tables[i].offsets);
        }
    }
    av_freep(&mxf->index_tables);

    if (mxf->growing_index_out) {
        /* safety net: normally mxf_growing_transition_to_closed() has already
         * deleted the sidecar (that IS the completion signal) by the time
         * close runs, but the caller may stop reading right after the last
         * packet without one further mxf_read_packet() call to trigger it */
        if (mxf->growing_file_closed)
            remove(mxf->growing_index_file);
        mxf_growing_release_sidecar_write(mxf);
    }
    mxf_growing_free_index(mxf);
    if (mxf->growing_h264_parser)
        av_parser_close(mxf->growing_h264_parser);
    avcodec_free_context(&mxf->growing_h264_avctx);

    return 0;
}

static int mxf_probe(const AVProbeData *p) {
    const uint8_t *bufp = p->buf;
    const uint8_t *end = p->buf + FFMIN(p->buf_size, RUN_IN_MAX + 1 + sizeof(mxf_header_partition_pack_key));

    if (p->buf_size < sizeof(mxf_header_partition_pack_key))
        return 0;

    /* Must skip Run-In Sequence and search for MXF header partition pack key SMPTE 377M 5.5 */
    end -= sizeof(mxf_header_partition_pack_key);

    for (; bufp < end;) {
        if (!((bufp[13] - 1) & 0xF2)){
            if (AV_RN32(bufp   ) == AV_RN32(mxf_header_partition_pack_key   ) &&
                AV_RN32(bufp+ 4) == AV_RN32(mxf_header_partition_pack_key+ 4) &&
                AV_RN32(bufp+ 8) == AV_RN32(mxf_header_partition_pack_key+ 8) &&
                AV_RN16(bufp+12) == AV_RN16(mxf_header_partition_pack_key+12))
                return bufp == p->buf ? AVPROBE_SCORE_MAX : AVPROBE_SCORE_MAX - 1;
            bufp ++;
        } else
            bufp += 10;
    }

    return 0;
}

/* rudimentary byte seek */
/* XXX: use MXF Index */
static int mxf_read_seek(AVFormatContext *s, int stream_index, int64_t sample_time, int flags)
{
    AVStream *st = s->streams[stream_index];
    int64_t seconds;
    MXFContext* mxf = s->priv_data;
    int64_t seekpos;
    int ret;
    MXFIndexTable *t;
    MXFTrack *source_track = st->priv_data;

    if (!source_track)
        return 0;

    /* if audio then truncate sample_time to EditRate */
    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
        sample_time = av_rescale_q(sample_time, st->time_base,
                                   av_inv_q(source_track->edit_rate));

    /* In growing mode a target past the live edge blocks until it arrives
     * rather than silently clamping. This must happen before anything takes a
     * pointer into mxf->index_tables, which the transition can rebuild. */
    if (mxf->growing) {
        ret = mxf_growing_wait_for_edit_unit(s, stream_index, sample_time);
        if (ret < 0)
            return ret;
    }

    /* growing VBR: the sidecar index is the only source of frame offsets */
    if (mxf->growing && mxf->growing_vbr_index) {
        MXFGrowingIndex *gi;
        MXFTrack *ref = mxf_growing_ref_track(s);

        mxf_growing_reload_sidecar(s);
        gi = mxf->growing_vbr_index;
        if (gi->nb_entries > 0 && mxf->nb_index_tables <= 0 && ref &&
            ref->edit_rate.num && source_track->edit_rate.num) {
            /* the sidecar is dense in REFERENCE-track edit units while
             * sample_time is in the seeking stream's, so rescale - without
             * this an audio seek indexes the array with a sample count */
            int64_t ref_eu = av_rescale_q(FFMAX(sample_time, 0),
                                          av_inv_q(source_track->edit_rate),
                                          av_inv_q(ref->edit_rate));

            if (ref_eu >= gi->nb_entries) {
                /* the wait established availability, so this means the sidecar
                 * was truncated under us. Never clamp. */
                av_log(s, AV_LOG_ERROR, "growing MXF: sidecar shrank during the "
                       "seek (%"PRId64" entries, wanted %"PRId64")\n",
                       gi->nb_entries, ref_eu);
                return AVERROR_EOF;
            }
            seekpos = avio_seek(s->pb, gi->offsets[ref_eu], SEEK_SET);
            if (seekpos < 0)
                return seekpos;
            avpriv_update_cur_dts(s, st, sample_time);
            mxf->current_klv_data = (KLVPacket){{0}};
            for (int i = 0; i < s->nb_streams; i++) {
                AVStream *cur_st = s->streams[i];
                MXFTrack *cur_track = cur_st->priv_data;
                if (cur_track) {
                    int64_t track_edit_unit = sample_time;
                    if (st != cur_st)
                        mxf_get_next_track_edit_unit(mxf, cur_track,
                                                     gi->offsets[ref_eu],
                                                     &track_edit_unit);
                    cur_track->sample_count = mxf_compute_sample_count(mxf, cur_st,
                                                                        track_edit_unit);
                }
            }
            return 0;
        }
    }

    if (mxf->nb_index_tables <= 0) {
        if (!s->bit_rate)
            return AVERROR_INVALIDDATA;
        if (sample_time < 0)
            sample_time = 0;
        seconds = av_rescale(sample_time, st->time_base.num, st->time_base.den);

        seekpos = avio_seek(s->pb, (s->bit_rate * seconds) >> 3, SEEK_SET);
        if (seekpos < 0)
            return seekpos;

        avpriv_update_cur_dts(s, st, sample_time);
        mxf->current_klv_data = (KLVPacket){{0}};
    } else {
        MXFPartition *partition;

        t = &mxf->index_tables[0];
        if (t->index_sid != source_track->index_sid) {
            int i;
            /* If the first index table does not belong to the stream, then find a stream which does belong to the index table */
            for (i = 0; i < s->nb_streams; i++) {
                MXFTrack *new_source_track = s->streams[i]->priv_data;
                if (new_source_track && new_source_track->index_sid == t->index_sid) {
                    sample_time = av_rescale_q(sample_time, new_source_track->edit_rate, source_track->edit_rate);
                    source_track = new_source_track;
                    st = s->streams[i];
                    break;
                }
            }
            if (i == s->nb_streams)
                return AVERROR_INVALIDDATA;
        }

        /* clamp above zero, else ff_index_search_timestamp() returns negative
         * this also means we allow seeking before the start */
        sample_time = FFMAX(sample_time, 0);

        if (t->fake_index) {
            /* The first frames may not be keyframes in presentation order, so
             * we have to advance the target to be able to find the first
             * keyframe backwards... */
            if (!(flags & AVSEEK_FLAG_ANY) &&
                (flags & AVSEEK_FLAG_BACKWARD) &&
                t->ptses[0] != AV_NOPTS_VALUE &&
                sample_time < t->ptses[0] &&
                (t->fake_index[t->ptses[0]].flags & AVINDEX_KEYFRAME))
                sample_time = t->ptses[0];

            /* behave as if we have a proper index */
            if ((sample_time = ff_index_search_timestamp(t->fake_index, t->nb_ptses, sample_time, flags)) < 0)
                return sample_time;
            /* get the stored order index from the display order index */
            sample_time += t->offsets[sample_time];
        } else {
            /* no IndexEntryArray (one or more CBR segments)
             * make sure we don't seek past the end */
            /* In growing mode mxf_growing_wait_for_edit_unit() has already
             * established that sample_time is available and
             * mxf_growing_refresh_duration() has re-published
             * original_duration, so clamping here would undo the wait. */
            if (!mxf->growing && source_track->original_duration > 0)
                sample_time = FFMIN(sample_time, source_track->original_duration - 1);
        }

        if (source_track->wrapping == UnknownWrapped)
            av_log(mxf->fc, AV_LOG_WARNING, "attempted seek in an UnknownWrapped essence\n");

        if ((ret = mxf_edit_unit_absolute_offset(mxf, t, sample_time, source_track->edit_rate, &sample_time, &seekpos, &partition, 1)) < 0)
            return ret;

        avpriv_update_cur_dts(s, st, sample_time);
        if (source_track->wrapping == ClipWrapped) {
            KLVPacket klv = partition->first_essence_klv;
            if (seekpos < klv.next_klv - klv.length || seekpos >= klv.next_klv) {
                av_log(mxf->fc, AV_LOG_ERROR, "attempted seek out of clip wrapped KLV\n");
                return AVERROR_INVALIDDATA;
            }
            mxf->current_klv_data = klv;
        } else {
            mxf->current_klv_data = (KLVPacket){{0}};
        }
        avio_seek(s->pb, seekpos, SEEK_SET);
    }

    // Update all tracks sample count
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *cur_st = s->streams[i];
        MXFTrack *cur_track = cur_st->priv_data;
        if (cur_track) {
            int64_t track_edit_unit = sample_time;
            if (st != cur_st)
                mxf_get_next_track_edit_unit(mxf, cur_track, seekpos, &track_edit_unit);
            cur_track->sample_count = mxf_compute_sample_count(mxf, cur_st, track_edit_unit);
        }
    }
    return 0;
}

static const AVOption options[] = {
    { "eia608_extract", "extract eia 608 captions from s436m track",
      offsetof(MXFContext, eia608_extract), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1,
      AV_OPT_FLAG_DECODING_PARAM },
    { "skip_essence_parse", "skip_essence_parse",
      offsetof(MXFContext, skip_essence_parse), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1,
      AV_OPT_FLAG_DECODING_PARAM },
    { "growing_poll_us", "microseconds to sleep between EOF polls in growing mode",
      offsetof(MXFContext, growing_poll_us), AV_OPT_TYPE_INT, {.i64 = 100000}, 1000, 10000000,
      AV_OPT_FLAG_DECODING_PARAM },
    { "growing_timeout_us", "microseconds without new data before returning EOF in "
      "growing mode (0 = wait forever; also applies during "
      "avformat_find_stream_info, and is required for a blocking seek to be able "
      "to give up)",
      offsetof(MXFContext, growing_timeout_us), AV_OPT_TYPE_INT64, {.i64 = 0}, 0, INT64_MAX,
      AV_OPT_FLAG_DECODING_PARAM },
    { "growing_index_stall_us", "microseconds the sidecar index may go without "
      "advancing before a blocked reader attempts to take the write lock; "
      "retried, never itself a give-up condition",
      offsetof(MXFContext, growing_index_stall_us), AV_OPT_TYPE_INT64, {.i64 = 2000000}, 0, INT64_MAX,
      AV_OPT_FLAG_DECODING_PARAM },
    { "growing_index_file", "local path to the growing-MXF sidecar index file; if given, "
      "the file is treated as growing only while no footer is present, and the "
      "sidecar is created and maintained for as long as it does",
      offsetof(MXFContext, growing_index_file), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0,
      AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass demuxer_class = {
    .class_name = "mxf",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEMUXER,
};

const FFInputFormat ff_mxf_demuxer = {
    .p.name         = "mxf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MXF (Material eXchange Format)"),
    .p.flags        = AVFMT_SEEK_TO_PTS | AVFMT_NOGENSEARCH,
    .p.priv_class   = &demuxer_class,
    .priv_data_size = sizeof(MXFContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = mxf_probe,
    .read_header    = mxf_read_header,
    .read_packet    = mxf_read_packet,
    .read_close     = mxf_read_close,
    .read_seek      = mxf_read_seek,
};

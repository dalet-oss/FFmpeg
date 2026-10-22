tests/data/dash_mpd_timing.mpd: TAG = GEN
tests/data/dash_mpd_timing.mpd: ffmpeg$(PROGSSUF)$(EXESUF) | tests/data
	$(RM) $(TARGET_PATH)/tests/data/dash_mpd_timing*
	$(M)$(TARGET_EXEC) $(TARGET_PATH)/$< -nostdin -loglevel error -re \
	-f lavfi -i "testsrc2=size=128x72:rate=1" -map 0:v \
	-c:v mpeg4 -g 1 -streaming 1 -window_size 5 \
	-availability_start_time_ms 1700000000123 -suggested_presentation_delay 2s \
	-f dash $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd & \
	pid=$$!; \
	i=0; \
	while ! test -s $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd && test $$i -lt 100; do \
	    sleep 0.1; \
	    i=$$((i + 1)); \
	done; \
	test -s $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd; \
	cp $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd $(TARGET_PATH)/tests/data/dash_mpd_timing.live.mpd; \
	kill $$pid 2>/dev/null || true; \
	wait $$pid 2>/dev/null || true; \
	cp $(TARGET_PATH)/tests/data/dash_mpd_timing.live.mpd $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd; \
	test -s $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd

FATE_DASHENC_LAVFI-$(call ALLYES, TESTSRC2_FILTER LAVFI_INDEV MPEG4_ENCODER DASH_MUXER MP4_MUXER FILE_PROTOCOL) += fate-dash-mpd-timing
fate-dash-mpd-timing: tests/data/dash_mpd_timing.mpd
fate-dash-mpd-timing: CMD = sed -n -e /suggestedPresentationDelay=/p -e /availabilityStartTime=/p $(TARGET_PATH)/tests/data/dash_mpd_timing.mpd
fate-dash-mpd-timing: CMP = diff

tests/data/dash_seek.mpd: TAG = GEN
tests/data/dash_seek.mpd: ffmpeg$(PROGSSUF)$(EXESUF) | tests/data
	$(M)$(TARGET_EXEC) $(TARGET_PATH)/$< -nostdin -loglevel error \
	-f lavfi -i "sine=sample_rate=16000:duration=3" \
	-c:a mp2fixed -b:a 8k -bitexact -f dash -seg_duration 1 \
	-init_seg_name dash_seek_init.mp4 -media_seg_name 'dash_seek_$$Number$$.m4s' $(TARGET_PATH)/$@

FATE_DASHENC_LAVFI-$(call ALLYES, SINE_FILTER ARESAMPLE_FILTER LAVFI_INDEV PCM_S16LE_DECODER MP2FIXED_ENCODER DASH_MUXER MP4_MUXER DASH_DEMUXER MOV_DEMUXER MP3_DECODER FILE_PROTOCOL FRAMECRC_MUXER PIPE_PROTOCOL) += fate-dash-seek-zero fate-dash-seek-reference
fate-dash-seek-zero fate-dash-seek-reference: tests/data/dash_seek.mpd
fate-dash-seek-zero: CMD = framecrc -ss 0 -i $(TARGET_PATH)/tests/data/dash_seek.mpd -c copy
fate-dash-seek-reference: CMD = framecrc -i $(TARGET_PATH)/tests/data/dash_seek.mpd -c copy
fate-dash-seek-reference: REF = $(SRC_PATH)/tests/ref/fate/dash-seek-zero

FATE_FFMPEG += $(FATE_DASHENC_LAVFI-yes)
fate-dashenc: $(FATE_DASHENC_LAVFI-yes)

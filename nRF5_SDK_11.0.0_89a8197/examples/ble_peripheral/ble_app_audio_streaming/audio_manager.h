#ifndef __AUDIO_MANAGER_H__
#define __AUDIO_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_util_platform.h"
#include "nrf.h"
#include "nrf_error.h"

typedef enum
{
    AUDIO_CODEC_BV32,
    AUDIO_CODEC_INVALID
} audio_codec_t;

typedef struct
{
    audio_codec_t codec;
} audio_init_t;

uint32_t audio_manager_init(audio_init_t * p_params);
bool     audio_manager_is_running(void);
uint32_t audio_manager_streaming_begin(void);
uint32_t audio_manager_streaming_begin_buffered(uint32_t frame_count);
uint32_t audio_manager_streaming_end(bool wait_for_fifo);
uint32_t audio_manager_play_test_tone(void);
uint32_t audio_manager_play_sample(void * p_sample, uint32_t len);
uint32_t audio_manager_pkt_process(void * p_pkt, uint32_t len);
uint32_t audio_manager_volume_get(float * p_volume);
uint32_t audio_manager_volume_set(float volume);

#endif /* __AUDIO_MANAGER_H__ */

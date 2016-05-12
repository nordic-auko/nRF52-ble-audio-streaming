#include "audio_manager.h"

#include "config.h"
#include "drv_sgtl5000.h"
#include "nrf_log.h"
#include "fifo.h"

#include "typedef.h"
#include "bv32cnst.h"
#include "bvcommon.h"
#include "bv32strct.h"
#include "bv32.h"
#include "utility.h"
#if G192BITSTREAM
#include "g192.h"
#else
#include "bitpack.h"
#endif

#define AUDIO_UPSAMPLING_FACTOR 4 /* Upsample from 8 kHz to 32 kHz: audio hardware seems to like this rate better */
#define AUDIO_FRAME_SIZE        FRSZ

static audio_codec_t m_audio_codec = AUDIO_CODEC_INVALID;

static struct
{
    struct BV32_Decoder_State ds;
} m_bv32_codec_params;

static struct
{
    bool      valid;
    uint8_t * p_sample;
    uint32_t  sample_len;
    uint32_t  sample_idx;
} m_sample_info;

static struct
{
    bool     buffering;
    uint32_t frames_left;
} m_frame_buffer_state;

static fifo_t   m_fifo_encoded_audio;
static int16_t  m_i2s_tx_buffer[AUDIO_FRAME_SIZE * AUDIO_UPSAMPLING_FACTOR * 2]; // Double-buffered
static bool     m_running;
static bool     m_stop_when_fifo_empty;

static bool codec_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
    bool ret;
    
    ret = true;
    
    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            // I2S TX buffer values requested
            {
                struct BV32_Bit_Stream bs;
                uint32_t               buf_len;
                uint8_t                packed_stream[20];
                int16_t                pcm_stream[AUDIO_FRAME_SIZE];
                
                if (m_sample_info.valid)
                {
                    // Get frame from sample buffer
                    
                    if ((m_sample_info.sample_idx + sizeof(packed_stream)) < m_sample_info.sample_len)
                    {
                        memcpy(packed_stream, &m_sample_info.p_sample[m_sample_info.sample_idx], sizeof(packed_stream));
                        m_sample_info.sample_idx += sizeof(packed_stream);
                        buf_len                   = sizeof(packed_stream);
                        ret                       = true; // Continue streaming in case of buffer underrun
                    }
                    else
                    {
                        // End of buffer reached. Stop playback
                        m_sample_info.valid = false;
                        m_running           = false;
                        ret                 = false;
                        buf_len             = 0;
                    }
                }
                else if (!m_frame_buffer_state.buffering)
                {
                    // Get frame from streaming FIFO
                    
                    APP_ERROR_CHECK_BOOL(p_evt->param.tx_buf_req.number_of_words == ((AUDIO_FRAME_SIZE * AUDIO_UPSAMPLING_FACTOR * sizeof(int16_t))/ sizeof(uint32_t)));
                    
                    buf_len = sizeof(packed_stream);
                    
                    CRITICAL_REGION_ENTER();
                    fifo_get_pkt(&m_fifo_encoded_audio, packed_stream, &buf_len);
                    CRITICAL_REGION_EXIT();
                }
                else
                {
                    buf_len = 0;
                    ret     = true;
                }
                
                if ((buf_len != sizeof(packed_stream)) && m_stop_when_fifo_empty)
                {
                    // End of buffer reached. Stop playback
                    m_sample_info.valid    = false;
                    m_running              = false;
                    ret                    = false;
                    m_stop_when_fifo_empty = false;
                    buf_len                = 0;
                    memset(p_evt->param.tx_buf_req.p_data_to_send, 0, p_evt->param.tx_buf_req.number_of_words * sizeof(uint32_t)); 
                    memset(&m_frame_buffer_state, 0, sizeof(m_frame_buffer_state));
                    return false;
                }
                else if (buf_len != sizeof(packed_stream))
                {
                    // No data to process: set to 0
                    memset(p_evt->param.tx_buf_req.p_data_to_send, 0, p_evt->param.tx_buf_req.number_of_words * sizeof(uint32_t)); 
                    return ret;
                }
                
                BV32_BitUnPack(packed_stream, &bs);
                BV32_Decode(&bs, &m_bv32_codec_params.ds, pcm_stream);
//                BV32_Decode(&bs, &m_bv32_codec_params.ds, (short*)p_evt->param.tx_buf_req.p_data_to_send);
                
                // Upsample the decompressed audio (because audio hardware requirements)
                for (int i = 0, pcm_stream_idx = 0; i < (AUDIO_FRAME_SIZE * AUDIO_UPSAMPLING_FACTOR); i += AUDIO_UPSAMPLING_FACTOR)
                {
                    for (int j = i; j < (i + AUDIO_UPSAMPLING_FACTOR); ++j)
                    {
                        ((int16_t*)p_evt->param.tx_buf_req.p_data_to_send)[j] = pcm_stream[pcm_stream_idx];
                    }
                    ++pcm_stream_idx;
                }
            }
            break;
    }
    
    return ret;
}

static uint32_t audio_pkt_process_bv32(void * p_packed_stream, uint32_t len)
{
    bool success;
    
    if (len != 20)
    {
        // BV32 frames are always 20 bytes
        return NRF_ERROR_INVALID_PARAM;
    }
    
    CRITICAL_REGION_ENTER();
    success = fifo_put_pkt(&m_fifo_encoded_audio, (uint8_t *) p_packed_stream, len);
    CRITICAL_REGION_EXIT();
    
    if (!success)
    {
        return NRF_ERROR_NO_MEM;
    }
    
    if (m_frame_buffer_state.buffering)
    {
        m_frame_buffer_state.frames_left -= 1;
        
        if (m_frame_buffer_state.frames_left == 0)
        {
            // Enough frames have been buffered
            m_frame_buffer_state.buffering = false;
        }
    }
    
    return NRF_SUCCESS;
}

uint32_t audio_manager_init(audio_init_t * p_params)
{
    drv_sgtl5000_init_t codec_params;
    
    if (p_params == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (p_params->codec == AUDIO_CODEC_BV32)
    {
        m_audio_codec = p_params->codec;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_running              = false;
    m_stop_when_fifo_empty = false;
    
    memset(&m_sample_info, 0, sizeof(m_sample_info));
    memset(&m_frame_buffer_state, 0, sizeof(m_frame_buffer_state));
    
    // Initialize audio decoder
    Reset_BV32_Decoder(&m_bv32_codec_params.ds);
    
    // Initialize FIFO 
    fifo_init(&m_fifo_encoded_audio);
    
    // Initialize codec hardware
    codec_params.i2s_tx_buffer     = (void*)m_i2s_tx_buffer;
    codec_params.i2s_tx_buffer_len = sizeof(m_i2s_tx_buffer);
    codec_params.evt_handler       = codec_driver_evt_handler;
    codec_params.fs                = DRV_SGTL5000_FS_31250HZ;
    
    return drv_sgtl5000_init(&codec_params);
}

uint32_t audio_manager_streaming_begin(void)
{
    uint32_t err_code;
    
    if (m_running)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    fifo_init(&m_fifo_encoded_audio);
    
    memset(m_i2s_tx_buffer, 0, sizeof(m_i2s_tx_buffer));
    
    switch (m_audio_codec)
    {
        case AUDIO_CODEC_BV32:
            Reset_BV32_Decoder(&m_bv32_codec_params.ds);
        
            err_code = drv_sgtl5000_start();
            break;
        
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }
    
    if (err_code == NRF_SUCCESS)
    {
        m_running = true;
    }
        
    return err_code;
}

uint32_t audio_manager_streaming_begin_buffered(uint32_t frame_count)
{
    if (frame_count == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_frame_buffer_state.frames_left = frame_count;
    m_frame_buffer_state.buffering   = true;
    
    return audio_manager_streaming_begin();
}

uint32_t audio_manager_play_test_tone(void)
{
    uint32_t err_code;
    
    if (m_running)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = drv_sgtl5000_start_1khz_test_tone();
    if (err_code == NRF_SUCCESS)
    {
        m_running = true;
    }
    
    return err_code;
}

uint32_t audio_manager_streaming_end(bool wait_for_fifo)
{
    uint32_t err_code;
    
    if (!m_running)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (!wait_for_fifo)
    {    
        err_code = drv_sgtl5000_stop();
        if (err_code == NRF_SUCCESS)
        {
            m_running = false;
            memset(&m_frame_buffer_state, 0, sizeof(m_frame_buffer_state));
        }
    }
    else
    {
        m_stop_when_fifo_empty = true;
        err_code               = NRF_SUCCESS;
    }
    
    return err_code;
}

bool audio_manager_is_running(void)
{
    return m_running;
}

uint32_t audio_manager_play_sample(void * p_sample, uint32_t len)
{
    if (m_sample_info.valid)
    {
        // Already playing sample
        return NRF_ERROR_INVALID_STATE;
    }
    
    m_sample_info.p_sample   = p_sample;
    m_sample_info.sample_idx = 0;
    m_sample_info.sample_len = len;
    m_sample_info.valid      = true;
    
    memset(&m_frame_buffer_state, 0, sizeof(m_frame_buffer_state));
    
    if (!m_running)
    {
        return audio_manager_streaming_begin();
    }
    
    return NRF_SUCCESS;
}

uint32_t audio_manager_pkt_process(void * p_pkt, uint32_t len)
{
    uint32_t err_code;
    
    switch (m_audio_codec)
    {
        case AUDIO_CODEC_BV32:
            err_code = audio_pkt_process_bv32(p_pkt, len);
            break;
        
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }
    
    return err_code;
}

uint32_t audio_manager_volume_get(float * p_volume)
{
    return drv_sgtl5000_volume_get(p_volume);
}

uint32_t audio_manager_volume_set(float volume)
{
    return drv_sgtl5000_volume_set(volume);
}

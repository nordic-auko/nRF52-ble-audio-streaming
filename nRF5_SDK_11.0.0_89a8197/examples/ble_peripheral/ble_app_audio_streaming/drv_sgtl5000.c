#include "drv_sgtl5000.h"

#include <string.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_soc.h"

#define SGTL5000_EGU_TASK_STREAMING_STOP 0

#define SGTL5000_SINE_TABLE_LEN 32

const static int16_t m_1khz_sine_table[SGTL5000_SINE_TABLE_LEN] = 
    {-16384, -13086, -9923,  -7024,  -4509,  -2480,  -1020,  -189, 
     -21,    -523,   -1674,  -3428,  -5712,  -8433,  -11479, -14726, 
     -18042, -21289, -24335, -27056, -29340, -31094, -32245, -32747, 
     -32579, -31748, -30288, -28259, -25744, -22845, -19682, -16384};

static nrf_drv_twi_t              m_twi_instance   = NRF_DRV_TWI_INSTANCE(DRV_SGTL5000_TWI_INSTANCE);
//static nrf_drv_timer_t        m_timer_instance = NRF_DRV_TIMER_INSTANCE(DRV_SGTL5000_TIMER_INSTANCE);
static drv_sgtl5000_handler_t     m_evt_handler;
static drv_sgtl5000_sample_freq_t m_fs;
static nrf_drv_i2s_config_t       m_i2s_config;
static float                      m_volume;

static struct
{
    uint32_t * i2s_tx_buffer;     
    uint32_t   i2s_tx_buffer_len; 
} m_i2s_configuration;

static volatile enum
{ 
    SGTL5000_TWI_TRANSFER_IDLE,
    SGTL5000_TWI_TRANSFER_PENDING,
    SGTL5000_TWI_TRANSFER_SUCCESS,
    SGTL5000_TWI_TRANSFER_FAILED
} m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;

static enum
{
    SGTL5000_STATE_UNINITIALIZED, /* Not initialized */
    SGTL5000_STATE_CONFIGURATION, /* Providing MCLK and configuring via TWI, but not streaming */
    SGTL5000_STATE_IDLE,          /* Initialized, but not running */
    SGTL5000_STATE_RUNNING,       /* Actively streaming audio */
    SGTL5000_STATE_RUNNING_1KHZ,  /* Actively streaming 1 kHz test tone */
} m_state = SGTL5000_STATE_UNINITIALIZED;

static void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{    
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_SUCCESS;
            break;
        
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_FAILED;
            break;
        
        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_FAILED;
            break;
    }
}

static void i2s_data_handler(uint32_t const * p_data_received, uint32_t * p_data_to_send, uint16_t number_of_words)
{
    drv_sgtl5000_evt_t evt;
    
    if (m_state == SGTL5000_STATE_RUNNING_1KHZ)
    {
        uint8_t * ptr;
        
        // Fill I2S buffer with 1 kHz tone data
        
        ptr = (uint8_t *) p_data_to_send;
        
        for (int i = 0; i < (number_of_words * 4); i += sizeof(m_1khz_sine_table))
        {
            memcpy(&ptr[i], m_1khz_sine_table, sizeof(m_1khz_sine_table));
        }
        
        return;
    }
    else if (m_state != SGTL5000_STATE_RUNNING)
    {
        // I2S only running in order to provide clock
        return;
    }
    
    if (p_data_to_send != NULL)
    {
        bool continue_running;
        
        // Request for I2S data to transmit
        evt.evt                              = DRV_SGTL5000_EVT_I2S_TX_BUF_REQ;
        evt.param.tx_buf_req.number_of_words = number_of_words;
        evt.param.tx_buf_req.p_data_to_send  = p_data_to_send;
        
        continue_running = m_evt_handler(&evt);
        
        if (!continue_running)
        {
            DRV_SGTL5000_EGU_INSTANCE->TASKS_TRIGGER[SGTL5000_EGU_TASK_STREAMING_STOP] = 1;
            
        }
    }
}

void DRV_SGTL5000_EGU_IRQHandler(void)
{
    if (DRV_SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] != 0)
    {
        DRV_SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] = 0;
        
        nrf_drv_i2s_stop();
        
        m_state = SGTL5000_STATE_IDLE;
    }
}

static bool sgtl5000_mclk_high_enough_for_twi(void)
{
    if (m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4 ||
        m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV3 ||
        m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2)
    {
        return true;
    }
    
    return false;
}

static void sgtl5000_mclk_enable(void)
{
    uint32_t             err_code;
    nrf_drv_i2s_config_t i2s_config;
    
    // Initialize I2S interface with MCLK >= 8 MHz in order to provide MCLK for initial SGTL5000 register access
    memcpy(&i2s_config, &m_i2s_config, sizeof(m_i2s_config));
    
    i2s_config.sdout_pin = NRF_DRV_I2S_PIN_NOT_USED;
    i2s_config.sdin_pin  = NRF_DRV_I2S_PIN_NOT_USED;
    i2s_config.mck_setup = NRF_I2S_MCK_32MDIV4;
    
    err_code = nrf_drv_i2s_init(&i2s_config, i2s_data_handler);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        nrf_drv_i2s_uninit();
        err_code = nrf_drv_i2s_init(&i2s_config, i2s_data_handler);
    }
    
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_i2s_start(0, m_i2s_configuration.i2s_tx_buffer, (m_i2s_configuration.i2s_tx_buffer_len / sizeof(uint32_t)), 0);
    APP_ERROR_CHECK(err_code);
}

static void sgtl5000_mclk_disable(void)
{
    uint32_t             err_code;

    // Initialize I2S interface with all pins again

    nrf_drv_i2s_stop();
    
    nrf_drv_i2s_uninit();
    
    err_code = nrf_drv_i2s_init(&m_i2s_config, i2s_data_handler);
    APP_ERROR_CHECK(err_code);
}

static uint32_t sgtl5000_register_read(uint16_t reg_addr, uint16_t * p_reg_data, bool blocking)
{
    nrf_drv_twi_xfer_desc_t twi_xfer;
    uint32_t                twi_flags;
    uint32_t                err_code;
    uint8_t                 reg_addr_buf[2];
    uint8_t                 reg_data_buf[2];
    
    reg_addr_buf[0] = (reg_addr >> 8) & 0xFF;
    reg_addr_buf[1] = (reg_addr)      & 0xFF;
    twi_flags       = NRF_DRV_TWI_FLAG_REPEATED_XFER;
    
    twi_xfer.address          = DRV_SGTL5000_TWI_ADDR;
    twi_xfer.type             = NRF_DRV_TWI_XFER_TXRX;
    twi_xfer.primary_length   = sizeof(reg_addr_buf);
    twi_xfer.p_primary_buf    = reg_addr_buf;
    twi_xfer.secondary_length = sizeof(reg_data_buf);
    twi_xfer.p_secondary_buf  = reg_data_buf;
    
    memset(reg_data_buf, 0, sizeof(reg_data_buf));
    
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
    
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &twi_xfer, twi_flags);
    
    if (err_code != NRF_SUCCESS)
    {
        sgtl5000_mclk_disable();
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
        return err_code;
    }
    
    while (blocking && (m_twi_transfer_state == SGTL5000_TWI_TRANSFER_PENDING))
    {
        __WFE();
    }
    
//    sgtl5000_mclk_disable();
    
    if (err_code != NRF_SUCCESS || m_twi_transfer_state != SGTL5000_TWI_TRANSFER_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
        return err_code;
    }
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
    
    *p_reg_data = (reg_data_buf[0] << 8 | reg_data_buf[1]);
    
    return NRF_SUCCESS;
}

static uint32_t sgtl5000_register_write(uint16_t reg_addr, uint16_t reg_data, bool blocking)
{
    nrf_drv_twi_xfer_desc_t twi_xfer;
    uint32_t                twi_flags;
    uint32_t                err_code;
    uint8_t                 write_buf[4];
    
    write_buf[0] = (reg_addr >> 8) & 0xFF;
    write_buf[1] = (reg_addr)      & 0xFF;
    write_buf[2] = (reg_data >> 8) & 0xFF;
    write_buf[3] = (reg_data)      & 0xFF;
    twi_flags    = 0;
    
    twi_xfer.address          = DRV_SGTL5000_TWI_ADDR;
    twi_xfer.type             = NRF_DRV_TWI_XFER_TX;
    twi_xfer.primary_length   = sizeof(write_buf);
    twi_xfer.p_primary_buf    = write_buf;
    
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
    
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &twi_xfer, twi_flags);
    
    if (err_code != NRF_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
        return err_code;
    }
    
    while (blocking && (m_twi_transfer_state == SGTL5000_TWI_TRANSFER_PENDING))
    {
        __WFE();
    }
    
    if (err_code != NRF_SUCCESS || m_twi_transfer_state != SGTL5000_TWI_TRANSFER_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
        return err_code;
    }
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
    
    return NRF_SUCCESS;
}

static void sgtl5000_register_write_verify(uint16_t reg_addr, uint16_t reg_data, uint16_t ro_mask)
{
    uint16_t read_value;
    
    NRF_LOG_PRINTF("Writing 0x%04x to register 0x%04x\r\n", reg_data, reg_addr);
    
    do
    {
        nrf_delay_us(50);
        sgtl5000_register_write(reg_addr, reg_data, true);
        nrf_delay_us(50);
        sgtl5000_register_read(reg_addr, &read_value, true);
    } while ((read_value & ro_mask) != reg_data);
}

uint32_t drv_sgtl5000_init(drv_sgtl5000_init_t * p_params)
{
    uint32_t                err_code;
    uint16_t                chip_id;
    
    NRF_LOG_PRINTF("drv_sgtl5000_init()\r\n");
    
    if (current_int_priority_get() < DRV_SGTL5000_TWI_IRQPriority)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    
    if (p_params->i2s_tx_buffer     == 0 ||
        p_params->i2s_tx_buffer_len == 0 ||
        p_params->evt_handler       == 0 ||
        p_params->fs                != DRV_SGTL5000_FS_31250HZ)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    DRV_SGTL5000_EGU_INSTANCE->INTENCLR = 0xFFFFFFFF;
    DRV_SGTL5000_EGU_INSTANCE->INTENSET = 0x0000FFFF;
    
    NVIC_ClearPendingIRQ(DRV_SGTL5000_EGU_IRQn);
    NVIC_SetPriority(DRV_SGTL5000_EGU_IRQn, DRV_SGTL5000_EGU_IRQPriority);
    NVIC_EnableIRQ(DRV_SGTL5000_EGU_IRQn);
    
    // Update configuration
    m_fs                                  = p_params->fs;
    m_evt_handler                         = p_params->evt_handler;
    m_i2s_configuration.i2s_tx_buffer     = p_params->i2s_tx_buffer;
    m_i2s_configuration.i2s_tx_buffer_len = p_params->i2s_tx_buffer_len;
    
    // Initialize TWI interface 
    nrf_drv_twi_config_t twi_config = {
        .frequency          = DRV_SGTL5000_TWI_FREQ,
        .interrupt_priority = DRV_SGTL5000_TWI_IRQPriority,
        .scl                = DRV_SGTL5000_TWI_PIN_SCL,
        .sda                = DRV_SGTL5000_TWI_PIN_SDA};
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_config, twi_event_handler, 0);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    nrf_drv_twi_enable(&m_twi_instance);
    
    // Disable pull-up resistors on SCL and SDA (already mounted on audio board)
    nrf_gpio_cfg(
        DRV_SGTL5000_TWI_PIN_SCL, 
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);
    
    nrf_gpio_cfg(
        DRV_SGTL5000_TWI_PIN_SDA, 
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);
    
    // Initialize I2S 
    m_i2s_config.sck_pin      = DRV_SGTL5000_I2S_PIN_BCLK;
    m_i2s_config.lrck_pin     = DRV_SGTL5000_I2S_PIN_LRCLK;
    m_i2s_config.mck_pin      = DRV_SGTL5000_I2S_PIN_MCLK;
    m_i2s_config.sdout_pin    = DRV_SGTL5000_I2S_PIN_TX;
    m_i2s_config.sdin_pin     = DRV_SGTL5000_I2S_PIN_RX;
    m_i2s_config.irq_priority = DRV_SGTL5000_I2S_IRQPriority;
    m_i2s_config.mode         = NRF_I2S_MODE_MASTER;
    m_i2s_config.format       = NRF_I2S_FORMAT_I2S;
    m_i2s_config.alignment    = NRF_I2S_ALIGN_LEFT;
    m_i2s_config.sample_width = NRF_I2S_SWIDTH_16BIT;
    m_i2s_config.channels     = NRF_I2S_CHANNELS_LEFT;

    switch (m_fs)
    {
        case DRV_SGTL5000_FS_31250HZ:
            m_i2s_config.mck_setup    = NRF_I2S_MCK_32MDIV4; // MCLK = 8 MHz
            m_i2s_config.ratio        = NRF_I2S_RATIO_256X;  // BCLK = 8 MHz / 256 = 31250 Hz 
            break;
        
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    
    
    err_code = nrf_drv_i2s_init(&m_i2s_config, i2s_data_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_state  = SGTL5000_STATE_IDLE;
    m_volume = -25.f;
    
    sgtl5000_mclk_enable();
    
    // Read ID register
    for (int i = 0; i < 5; ++i)
    {
        err_code = sgtl5000_register_read(DRV_SGTL5000_REGISTER_ADDR_CHIP_ID, &chip_id, true);
        if (err_code == NRF_SUCCESS && (chip_id & 0xFF00) == 0xA000)
        {
            break;
        }
    }

    if (err_code != NRF_SUCCESS || (chip_id & 0xFF00) != 0xA000)
    {
        sgtl5000_mclk_disable();
        return NRF_ERROR_NOT_FOUND;
    }

    // VDDD is externally driven with 1.8V
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER, 0x0020, 0xFFFF);

//    // VDDA & VDDIO both over 3.1V
//    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_LINREG_CTRL, 0x006C);

    // VAG=1.575, normal ramp, +12.5% bias current
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_REF_CTRL, 0x01F2, 0xFFFF);

    // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_CTRL, 0x0F22, 0xFFFF);

    // allow up to 125mA
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_SHORT_CTRL, 0x4446, 0xFFFF);

    // enable zero cross detectors
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 0x0137, 0xFFFF);

    // power up all digital stuff
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_DIG_POWER, 0x0073, 0xFFFF);
    
//    // Enable PLL: int divisor = 0x18, frac divisor = 0x49C
//    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_PLL_CTRL, ((0x18 << 10) | 0x49C), 0xFFFF);
    
//    // power up: lineout, hp, adc, dac,  pll, pll vco
//    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER, 0x45FF, 0xFFFF);

    // power up: lineout, hp, adc, dac
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER, 0x40FF, 0xFFFF);

    // default approx 1.3 volts peak-to-peak
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_VOL, 0x0F0F, 0xFFFF);

//    // sys_fs = 32 kHz, rate_mode = sys_fs / 4, mclk_freq = use PLL
//    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL, 0x0023, 0xFFFF);

    switch (m_fs)
    {
        case DRV_SGTL5000_FS_31250HZ:
            // sys_fs = 32 kHz, rate_mode = sys_fs, mclk_freq = 256*Fs
            sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL, 0x0000, 0xFFFF);
//            // sys_fs = 32 kHz, rate_mode = sys_fs / 4, mclk_freq = 256*Fs
//            sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL, 0x0020, 0xFFFF);
            break;
                
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }

//    // SCLK=32*Fs, 16bit, I2S format, Master mode
//    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_I2S_CTRL, 0x01B0, 0xFFFF);

    // SCLK=32*Fs, 16bit, I2S format, Slave mode
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_I2S_CTRL, 0x0130, 0xFFFF);
    
    // ADC->I2S, I2S->DAC
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_SSS_CTRL, 0x0010, 0xFFFF); 
    
    // disable dac mute
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ADCDAC_CTRL, 0x0000, 0x030F);
    
    // digital gain, 0dB
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_DAC_VOL, 0x3C3C, 0xFFFF);
    
    // set analog gain (50% of max level)
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, ((0x4A << 8) | 0x4A), 0xFFFF);
    
    // enable zero cross detectors. Unmute HP
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 0x0026, 0xFFFF);
    
    sgtl5000_mclk_disable();
    
    m_state = SGTL5000_STATE_IDLE;

    return NRF_SUCCESS;
}

uint32_t drv_sgtl5000_start(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING;
        
        nrf_drv_i2s_start(0, m_i2s_configuration.i2s_tx_buffer, (m_i2s_configuration.i2s_tx_buffer_len / sizeof(uint32_t)), 0);
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t drv_sgtl5000_start_1khz_test_tone(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING_1KHZ;
        
        nrf_drv_i2s_start(0, m_i2s_configuration.i2s_tx_buffer, (m_i2s_configuration.i2s_tx_buffer_len / sizeof(uint32_t)), 0);
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t drv_sgtl5000_stop(void)
{
    if (m_state == SGTL5000_STATE_RUNNING ||
        m_state == SGTL5000_STATE_RUNNING_1KHZ)
    {
        nrf_drv_i2s_stop();
        
        m_state = SGTL5000_STATE_IDLE;
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t drv_sgtl5000_volume_set(float volume_db)
{
    bool    start_mclk;
    float   volume_float;
    uint8_t volume_right;
    uint8_t volume_left;
    
    if (m_state != SGTL5000_STATE_IDLE && !sgtl5000_mclk_high_enough_for_twi())
    {
        // Need fast MCLK to read/write configuration registers.
        // Cannot do this while streaming audio with 2 MHz MCLK
        return NRF_ERROR_INVALID_STATE;
    }
    
    // Valid range for analog amplifier: -51.5 to +12 dB in .5 dB steps
    if (volume_db > 12.f ||
        volume_db < -51.5f)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_volume = volume_db;
    
    // Value 0x00 = 12 dB (max)
    // Value 0x7F = -51.5 dB (min)
    
    volume_float = volume_db + 51.5f;
    APP_ERROR_CHECK_BOOL(volume_float >= 0.f);
    
    volume_right = (uint8_t) 0x7F - ((volume_float / 63.5f) * 127.f);
    volume_left  = volume_right;
    
    APP_ERROR_CHECK_BOOL(volume_right <= 0x7F);
    APP_ERROR_CHECK_BOOL(volume_left <= 0x7F);
    
#ifdef DEBUG
    {
        char str[100];
        
        sprintf(str, "Setting volume to %f (0x%02x)\r\n", volume_db, volume_right);
        NRF_LOG_PRINTF(str);
    }
#endif 
    
    if (m_state == SGTL5000_STATE_IDLE)
    {
        sgtl5000_mclk_enable();
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, ((volume_right << 8) | volume_left), 0xFFFF);
        sgtl5000_mclk_disable();
    }
    else
    {
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, ((volume_right << 8) | volume_left), 0xFFFF);
    }
    
    return NRF_SUCCESS;
}

uint32_t drv_sgtl5000_volume_get(float * p_volume_db)
{
    if (m_state == SGTL5000_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    *p_volume_db = m_volume;
    
    return NRF_SUCCESS;
}

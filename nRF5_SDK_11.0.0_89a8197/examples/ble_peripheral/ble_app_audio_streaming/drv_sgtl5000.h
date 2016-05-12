#ifndef __DRV_SGTL5000_H__
#define __DRV_SGTL5000_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_util_platform.h"
#include "config.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"

// Utility EGU interrupt
#define DRV_SGTL5000_EGU_INSTANCE    NRF_EGU3
#define DRV_SGTL5000_EGU_IRQn        SWI3_EGU3_IRQn
#define DRV_SGTL5000_EGU_IRQHandler  SWI3_EGU3_IRQHandler
#define DRV_SGTL5000_EGU_IRQPriority APP_IRQ_PRIORITY_LOWEST

// TWI settings
#define DRV_SGTL5000_TWI_ADDR        0x0A /* Note: has been right-shifted such that R/W bit is not part of this value*/
#define DRV_SGTL5000_TWI_FREQ        NRF_TWI_FREQ_250K
#define DRV_SGTL5000_TWI_IRQPriority APP_IRQ_PRIORITY_HIGH

/* Pins follow the arduino shield for factor, so these are unlikely to change */
// TWI pin mapping
#define DRV_SGTL5000_TWI_PIN_SCL   27
#define DRV_SGTL5000_TWI_PIN_SDA   26

// I2S Settings
#define DRV_SGTL5000_I2S_IRQPriority  DRV_SGTL5000_EGU_IRQPriority

// I2S pin mapping
#define DRV_SGTL5000_I2S_PIN_MCLK  23
#define DRV_SGTL5000_I2S_PIN_BCLK  20
#define DRV_SGTL5000_I2S_PIN_LRCLK 22
#define DRV_SGTL5000_I2S_PIN_RX    25
#define DRV_SGTL5000_I2S_PIN_TX    24

// Register definitions
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ID            0x0000
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_DIG_POWER     0x0002
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL      0x0004
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_I2S_CTRL      0x0006
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_SSS_CTRL      0x000A
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ADCDAC_CTRL   0x000E
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_DAC_VOL       0x0010
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL   0x0022
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL      0x0024
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINREG_CTRL   0x0026
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_REF_CTRL      0x0028
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_CTRL 0x002C
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_VOL  0x002E
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER     0x0030
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_PLL_CTRL      0x0032
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_SHORT_CTRL    0x003C


typedef enum
{
    DRV_SGTL5000_EVT_I2S_TX_BUF_REQ, /* Request for I2S TX buffer */
} drv_sgtl5000_evt_type_t;

typedef enum
{
  DRV_SGTL5000_FS_31250HZ,  
} drv_sgtl5000_sample_freq_t;

typedef struct
{
    drv_sgtl5000_evt_type_t evt;
    union
    {
        struct
        {
            uint32_t * p_data_to_send;  /* Pointer to buffer that should be filled  */
            uint16_t   number_of_words; /* Buffer size in number of Words (32 bits) */
        } tx_buf_req;
    } param;
} drv_sgtl5000_evt_t;

typedef bool (* drv_sgtl5000_handler_t)(drv_sgtl5000_evt_t * p_evt);

typedef struct
{
    drv_sgtl5000_handler_t     evt_handler;
    drv_sgtl5000_sample_freq_t fs;
    void *                     i2s_tx_buffer;     /* Pointer to I2S TX double-buffer (should be 2 x uncompressed frame size) */
    uint32_t                   i2s_tx_buffer_len; /* Size of buffer (in bytes) */ 
} drv_sgtl5000_init_t;

uint32_t drv_sgtl5000_init(drv_sgtl5000_init_t * p_params);
uint32_t drv_sgtl5000_start(void);
uint32_t drv_sgtl5000_start_1khz_test_tone(void);
uint32_t drv_sgtl5000_stop(void);
uint32_t drv_sgtl5000_volume_set(float volume_db);
uint32_t drv_sgtl5000_volume_get(float * p_volume_db);

#endif /* __DRV_SGTL5000_H__ */

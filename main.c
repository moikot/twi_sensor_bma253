/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>

#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* MBA253 address when SDO is connected to GND - 0x18. */
#define BMA253_ADDR           0x18U

/* MBA253 registries */

#define BMA253_INT_EN_0_REG   0x16U
#define BMA253_INT_MAP_0_REG  0x19U
#define BMA253_INT_9_REG      0x2BU

/* Connected pins */
#define PIN_INT1  26 
#define PIN_SCL   18 
#define PIN_SDA   17

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief Function for setting MBA253 registry.
 */
void MBA253_set_reg(uint8_t reg, uint8_t value)
{
    uint8_t reg4[2] = {reg, value};
    m_xfer_done = false;
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, BMA253_ADDR, reg4, sizeof(reg4), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for setting MBA253 mode.
 */
void MBA253_set_mode(void)
{
    // Change tap threshold
    MBA253_set_reg(BMA253_INT_9_REG, 0x01);
  
    /* Enable signe tap interrupt on INT1. */
    MBA253_set_reg(BMA253_INT_EN_0_REG, 0x20);
    MBA253_set_reg(BMA253_INT_MAP_0_REG, 0x20);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bma253_config = {
       .scl                = PIN_SCL,
       .sda                = PIN_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_bma253_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static volatile int cnt = 0;
    // Count interruptions
    cnt++;
}

/**
 * @brief Initialize GPIO and enable interrupts
 */
void gpio_init(void) 
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    // Enable interruptions input
    err_code = nrf_drv_gpiote_in_init(PIN_INT1, &in_config, interrupt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_INT1, true);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    twi_init();
    gpio_init();

    MBA253_set_mode();

    while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);
    }
}

/** @} */

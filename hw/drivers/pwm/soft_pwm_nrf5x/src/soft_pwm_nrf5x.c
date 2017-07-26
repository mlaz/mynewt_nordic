/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <hal/hal_bsp.h>
#include <assert.h>
#include <os/os.h>
#include <errno.h>
#include <pwm/pwm.h>
#include <string.h>

/* Nordic headers */
#include <bsp.h>
#include <nrf.h>
#include <nrf_pwm.h>
#include <nrf_drv_pwm.h>
#include <nrf_drv_clock.h>
/* #include <bsp/cmsis_nvic.h> */
#include <app_timer.h>

//#include <app_error.h>
#include <app_util_platform.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_timer.h>

/* extern void PWM0_IRQHandler(void); */

/* Mynewt Nordic driver */
#include "pwm_nrf52/pwm_nrf52.h"

//static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

struct nrf53_pwm_dev_global {
    bool in_use;
    bool playing;
    nrf_drv_pwm_t drv_instance;
    nrf_drv_pwm_config_t config;
    nrf_pwm_values_individual_t *duty_cycles;
};

/**
 * Open the NRF52 PWM device
 *
 * This function locks the device for access from other tasks.
 *
 * @param odev The OS device to open
 * @param wait The time in MS to wait.  If 0 specified, returns immediately
 *             if resource unavailable.  If OS_WAIT_FOREVER specified, blocks
 *             until resource is available.
 * @param arg  Argument provided by higher layer to open, in this case
 *             it can be a nrf_drv_pwm_config_t, to override the default
 *             configuration.
 *
 * @return 0 on success, non-zero on failure.
 */
static int
nrf5x_soft_pwm_open(struct os_dev *odev, uint32_t wait, void *arg)
{
    struct pwm_dev *dev;
    int stat = 0;
    int inst_id;
    dev = (struct pwm_dev *) odev;
    inst_id = dev->pwm_instance_id;

    if (os_started()) {
        stat = os_mutex_pend(&dev->pwm_lock, wait);
        if (stat != OS_OK) {
            return (stat);
        }
    }

    if (odev->od_flags & OS_DEV_F_STATUS_OPEN) {
        os_mutex_release(&dev->pwm_lock);
        stat = OS_EBUSY;
        return (stat);
    }

    stat = init_instance(inst_id, arg);
    if (stat) {
        return (stat);
    }

    return (0);
}

/**
 * Close the NRF52 PWM device.
 *
 * This function unlocks the device.
 *
 * @param odev The device to close.
 */
static int
nrf5x_soft_pwm_close(struct os_dev *odev)
{
    struct pwm_dev *dev;
    int inst_id;

    dev = (struct pwm_dev *) odev;
    inst_id = dev->pwm_instance_id;

    /* nrf_drv_pwm_uninit(&instances[inst_id].drv_instance); */

    if (os_started()) {
        os_mutex_release(&dev->pwm_lock);
    }

    return (0);
}

/**
 * Configure a channel on the PWM device.
 *
 * @param dev The device to configure.
 * @param cnum The channel number to configure.
 * @param data Driver specific configuration data for this channel.
 *
 * @return 0 on success, non-zero error code on failure.
 */
static int
nrf5x_soft_pwm_configure_channel(struct pwm_dev *dev,
                                 uint8_t cnum,
                                 void *data)
{

    return (0);
}

/**
 * Enable the PWM with specified duty cycle.
 *
 * This duty cycle is a fractional duty cycle where 0 == off, 65535=on,
 * and any value in between is on for fraction clocks and off
 * for 65535-fraction clocks.
 *
 * @param dev The device to configure.
 * @param cnum The channel number. The channel should be in use.
 * @param fraction The fraction value.
 *
 * @return 0 on success, negative on error.
 */
static int
nrf5x_soft_pwm_enable_duty_cycle(struct pwm_dev *dev,
                                 uint8_t cnum,
                                 uint16_t fraction)
{

    return (0);
}

/**
 * Disable the PWM channel, it will be marked as unconfigured.
 *
 * @param dev The device to configure.
 * @param cnum The channel number.
 *
 * @return 0 success, negative on error.
 */
static int
nrf5x_soft_pwm_disable(struct pwm_dev *dev, uint8_t cnum)
{
    return (0);
}

/**
 * This frequency must be between 1/2 the clock frequency and
 * the clock divided by the resolution. NOTE: This may affect
 * other PWM channels.
 *
 * @param dev The device to configure.
 * @param freq_hz The frequency value in Hz.
 *
 * @return A value is in Hz on success, negative on error.
 */
static int
nrf5x_soft_pwm_set_frequency(struct pwm_dev *dev, uint32_t freq_hz)
{
    return (EINVAL);
}

/**
 * Get the underlying clock driving the PWM device.
 *
 * @param dev
 *
 * @return value is in Hz on success, negative on error.
 */
static int
nrf5x_soft_pwm_get_clock_freq(struct pwm_dev *dev)
{
    return (6666);
}

/**
 * Get the resolution of the PWM in bits.
 *
 * @param dev The device to query.
 *
 * @return The value in bits on success, negative on error.
 */
static int
nrf5x_soft_pwm_get_resolution_bits(struct pwm_dev *dev)
{
    return (0);
}

/**
 * Callback to initialize an adc_dev structure from the os device
 * initialization callback.  This sets up a nrf52_pwm_device(), so
 * that subsequent lookups to this device allow us to manipulate it.
 */
int
nrf52_pwm_dev_init(struct os_dev *odev, void *arg)
{
    struct pwm_dev *dev;
    struct pwm_driver_funcs *pwm_funcs;

    dev = (struct pwm_dev *) odev;

    if (arg) {
        dev->pwm_chan_count = *((int*) arg);
    } else {
        dev->pwm_chan_count = 1;
    }
    //should we have a way to prevent to have more than one instance?

    os_mutex_init(&dev->pwm_lock);

    OS_DEV_SETHANDLERS(odev, nrf52_pwm_open, nrf52_pwm_close);

    pwm_funcs = &dev->pwm_funcs;
    pwm_funcs->pwm_configure_channel = nrf5x_soft_pwm_configure_channel;
    pwm_funcs->pwm_enable_duty_cycle = nrf5x_soft_pwm_enable_duty_cycle;
    pwm_funcs->pwm_set_frequency = nrf5x_soft_pwm_set_frequency;
    pwm_funcs->pwm_get_clock_freq = nrf5x_soft_pwm_get_clock_freq;
    pwm_funcs->pwm_get_resolution_bits = nrf5x_soft_pwm_get_resolution_bits;
    pwm_funcs->pwm_disable = nrf5x_soft_pwm_disable;

    /* NVIC_SetVector(PWM0_IRQn, (uint32_t) PWM0_IRQHandler); */
    return (0);
}

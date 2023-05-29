/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include <stdio.h>

// Pico
#include "pico/stdlib.h"

// For scheduling sensor measurements
#include "hardware/timer.h"

// Definitions for setting up usb device
#include "usb_hal.h"

#include "lsm6ds3.h"
#include "timeSens.h"
#include "sensAcqImpl.h"

// LEDs for status
#define LED_R   13
#define LED_G   14
#define LED_B   15

int main(void) {
    gpio_init_mask(1 << LED_R | 1 << LED_G | 1 << LED_B);
    gpio_set_dir_out_masked(1 << LED_R | 1 << LED_G | 1 << LED_B);
    gpio_put_masked(1 << LED_R | 1 << LED_G | 1 << LED_B, 1 << LED_R | 1 << LED_G | 1 << LED_B);

    // Initialize the IMU
    lsm6ds3_setup();

    // Initialize the Timer
    timeSens_setup();

    if(initDataAcq())
    {
        // Everything is interrupt driven so just loop here
        while (1)
        {
            tight_loop_contents();
        }
    }

    return 0;
}

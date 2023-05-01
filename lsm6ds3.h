/**
 * @file LSM6DS3.hpp
 * Driver for the ST LSM6DS3 MEMS accelerometer / gyroscope connected via SPI.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "sensAcqImpl.h"

#ifndef LSM6DS3_HEADER
#define LSM6DS3_HEADER

// Bit definitions
#define Bit0    (1 << 0)
#define Bit1    (1 << 1)
#define Bit2    (1 << 2)
#define Bit3    (1 << 3)
#define Bit4    (1 << 4)
#define Bit5    (1 << 5)
#define Bit6    (1 << 6)
#define Bit7    (1 << 7)

/* register addresses: XL: accel, G: gyro, TEMP: temp */
#define ADDR_WHO_AM_I 0x0F
#define WHO_I_AM 0x6A

#define CTRL1_XL 0x10 // Linear acceleration sensor, Control Register 1.
#define CTRL2_G 0x11  // Angular rate sensor, Control Register 2.
#define CTRL3_C 0x12  // Control register 3.
#define CTRL4_C 0x13  // Control register 4.
#define CTRL5_C 0x14  // Control register 5.
#define CTRL6_C 0x15  // Control register 6.
#define CTRL7_G 0x16  // Angular rate sensor, Control Register 7.
#define CTRL8_XL 0x17 // Linear acceleration sensor, Control Register 8.
#define CTRL9_XL 0x18 // Linear acceleration sensor, Control Register 9.
#define CTRL10_C 0x19 // Control register 10.

#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21

#define STATUS_REG 0x1E // Only one STATUS_REG as compared to lsm9ds1

#define OUT_X_L_G 0x22
#define OUT_X_H_G 0x23
#define OUT_Y_L_G 0x24
#define OUT_Y_H_G 0x25
#define OUT_Z_L_G 0x26
#define OUT_Z_H_G 0x27

#define OUT_X_L_XL 0x28
#define OUT_X_H_XL 0x29
#define OUT_Y_L_XL 0x2A
#define OUT_Y_H_XL 0x2B
#define OUT_Z_L_XL 0x2C
#define OUT_Z_H_XL 0x2D

#define CTRL1_XL_ODR_XL_BITS (Bit7 | Bit6 | Bit5 | Bit4)
#define CTRL1_XL_ODR_XL_26Hz (Bit5)
#define CTRL1_XL_ODR_XL_52Hz (Bit5 | Bit4)
#define CTRL1_XL_ODR_XL_104Hz (Bit6)
#define CTRL1_XL_ODR_XL_208Hz (Bit6 | Bit4)
#define CTRL1_XL_ODR_XL_416Hz (Bit6 | Bit5)
#define CTRL1_XL_ODR_XL_833Hz (Bit6 | Bit5 | Bit4)

#define CTRL1_XL_FS_XL_BITS (Bit3 | Bit2)
#define CTRL1_XL_FS_XL_2g (0)
#define CTRL1_XL_FS_XL_4g (Bit3)
#define CTRL1_XL_FS_XL_8g (Bit3 | Bit2)
#define CTRL1_XL_FS_XL_16g (Bit2)

#define CTRL2_G_ODR_G_BITS (Bit7 | Bit6 | Bit5 | Bit4)
#define CTRL2_G_ODR_G_26Hz (Bit5)
#define CTRL2_G_ODR_G_52Hz (Bit5 | Bit4)
#define CTRL2_G_ODR_G_104Hz (Bit6)
#define CTRL2_G_ODR_G_208Hz (Bit6 | Bit4)
#define CTRL2_G_ODR_G_416Hz (Bit6 | Bit5)
#define CTRL2_G_ODR_G_833Hz (Bit6 | Bit5 | Bit4)

#define CTRL2_G_FS_G_BITS (Bit3 | Bit2)
#define CTRL2_G_FS_G_125dps (Bit1)
#define CTRL2_G_FS_G_250dps (0)
#define CTRL2_G_FS_G_500dps (Bit2)
#define CTRL2_G_FS_G_1000dps (Bit3)
#define CTRL2_G_FS_G_2000dps (Bit3 | Bit2)

#define CTRL3_C_BDU (Bit6)
#define CTRL3_C_IF_INC (Bit2)
#define CTRL3_C_BLE (Bit1)
#define CTRL3_C_SW_RESET (Bit0)

#define CTRL4_C_I2C_disable (Bit2)

#define STATUS_REG_TDA (Bit2)
#define STATUS_REG_GDA (Bit1)
#define STATUS_REG_XLDA (Bit0)

/* default values for this device */
#define LSM6DS3_ACCEL_DEFAULT_RANGE_G 16
#define LSM6DS3_ACCEL_DEFAULT_ODR 26

#define LSM6DS3_GYRO_DEFAULT_RANGE_DPS 2000
#define LSM6DS3_GYRO_DEFAULT_ODR 26

// Pin Definitions
#define PIN_MISO    8
#define PIN_MOSI    11
#define PIN_SCK     10
#define PIN_CS      9

// SPI Definitions
#define SPI_PORT    spi1
#define READ_BIT    (1 << 7)

// Function Definitions
void lsm6ds3_setup();
bool lsm6ds3_update_data(repeating_timer_t* tmr);

#endif
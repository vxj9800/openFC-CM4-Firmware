#include "lsm6ds3.h"

// Data Definitions
static struct
{
    uint8_t status;
    int16_t temperature;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
} data_report;
static bool lsm6ds3_drdy = false;

// Repeating timer global for calling the update data function repeatedly
static repeating_timer_t t_lsm6ds3;

static inline void cs_select()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0); // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}

static void write_registers(uint8_t *buf, uint16_t len)
{
    // For this device, the first value on the buffer has to be the register from which
    // the writing should be started. And, the rest of the values could be the values
    // that needs to be written to the registers.
    cs_select();
    spi_write_blocking(SPI_PORT, buf, len);
    cs_deselect();
}

void lsm6ds3_setup()
{
    // Setup the GPIOs for SPI communication
    spi_init(SPI_PORT, 10 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    uint8_t buf[2];

    // Disable I2C
    buf[0] = CTRL4_C;
    buf[1] = CTRL4_C_I2C_disable;
    write_registers(buf, 2);

    // Enable Block Data Update and auto address increment
    buf[0] = CTRL3_C;
    buf[1] = CTRL3_C_BDU | CTRL3_C_IF_INC;
    write_registers(buf, 2);

    // Set accelerometer range
    buf[0] = CTRL1_XL;
    read_registers(buf[0], &buf[1], 1);
    buf[1] &= CTRL1_XL_FS_XL_BITS;
    buf[1] |= CTRL1_XL_FS_XL_16g;
    write_registers(buf, 2);

    // Set accelerometer samplerate
    buf[0] = CTRL1_XL;
    read_registers(buf[0], &buf[1], 1);
    buf[1] &= CTRL1_XL_ODR_XL_BITS;
    buf[1] |= CTRL1_XL_ODR_XL_833Hz;
    write_registers(buf, 2);

    // Set gyroscope range
    buf[0] = CTRL2_G;
    read_registers(buf[0], &buf[1], 1);
    buf[1] &= CTRL2_G_FS_G_BITS;
    buf[1] |= CTRL2_G_FS_G_2000dps;
    write_registers(buf, 2);

    // Set gyroscope samplerate
    buf[0] = CTRL2_G;
    read_registers(buf[0], &buf[1], 1);
    buf[1] &= CTRL2_G_ODR_G_BITS;
    buf[1] |= CTRL2_G_ODR_G_833Hz;
    write_registers(buf, 2);

    // Schedule the IMU to be measured at the defined frequency
    add_repeating_timer_us(1200, lsm6ds3_update_data, NULL, &t_lsm6ds3);

    // Register the IMU with the usb data communication system
    register_sensor(5,DTYPE_INT16,6,833,(uint8_t *)&data_report.gx,&lsm6ds3_drdy);
}

bool lsm6ds3_update_data(repeating_timer_t *tmr)
{
    gpio_xor_mask(1 << 13);
    if (!lsm6ds3_drdy)
    {
        read_registers(STATUS_REG, (uint8_t *)&data_report, sizeof(data_report));

        if (data_report.status & (STATUS_REG_GDA | STATUS_REG_XLDA))
            lsm6ds3_drdy = true;
    }
    return 1;
}
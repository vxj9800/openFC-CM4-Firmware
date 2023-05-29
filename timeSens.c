#include "timeSens.h"

// Data Definitions
static uint64_t time;
static bool timeSens_drdy = false;

// Repeating timer global for calling the update data function repeatedly
static repeating_timer_t t_timeSens;

void timeSens_setup()
{
    // Schedule the time to be measured at the defined frequency
    add_repeating_timer_us(1000, timeSens_update_data, NULL, &t_timeSens);

    // Register the timeSens with the usb data communication system
    register_sensor(0, DTYPE_UINT64, 1, 1000, (uint8_t *)&time, &timeSens_drdy);
}

bool timeSens_update_data(repeating_timer_t *tmr)
{
    if (!timeSens_drdy)
    {
        time = time_us_64();
        timeSens_drdy = true;
    }
    return 1;
}
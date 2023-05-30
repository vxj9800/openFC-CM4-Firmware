#include "timeSens.h"

// Data Definitions
static bool timeSensLo_drdy = true, timeSensHi_drdy = true;

// Repeating timer global for calling the update data function repeatedly
static repeating_timer_t t_timeSens;

void timeSens_setup()
{
    // Schedule the time to be measured at the defined frequency
    add_repeating_timer_us(1000, timeSens_update_data, NULL, &t_timeSens);

    // Register the timeSens with the usb data communication system
    register_sensor(0, DTYPE_UINT32, 1, 1000, (uint8_t *)&timer_hw->timelr, &timeSensLo_drdy);
    register_sensor(1, DTYPE_UINT32, 1, 1000, (uint8_t *)&timer_hw->timehr, &timeSensHi_drdy);
}

bool timeSens_update_data(repeating_timer_t *tmr)
{
    timeSensLo_drdy = true;
    timeSensHi_drdy = true;
    return 1;
}
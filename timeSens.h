/**
 * @file timeSens.hpp
 * Collect HRT value
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "sensAcqImpl.h"

#ifndef TIMESENSOR_HEADER
#define TIMESENSOR_HEADER

// Function Definitions
void timeSens_setup();
bool timeSens_update_data(repeating_timer_t* tmr);

#endif
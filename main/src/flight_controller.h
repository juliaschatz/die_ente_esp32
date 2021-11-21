#pragma once

#define CONTROL_TIMEOUT 50 // ticks
#define NVS_AHRS_NS "ahrs"
#define NVS_AHRS_KEY_CAL "cal"
#define AHRS_CAL_SIZE 39

void flightControllerTask(void * arg);
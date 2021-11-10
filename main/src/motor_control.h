#pragma once

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

#define OFF_PULSE_US 1000
#define MIN_PULSE_US 1080
#define MAX_PULSE_US 2000
#define MAX_POWER 0.51 // Safe limit

class Motor {
  public:
  void setSpeed(float speed);
  Motor(int gpio_pwm);
  void begin();

  private:
  // MCPWM addresses
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
  mcpwm_io_signals_t op;
  int gpio_pwm;

  static void initUnit(mcpwm_unit_t unit, mcpwm_timer_t timer);
  static mcpwm_unit_t next_unit;
  static mcpwm_io_signals_t next_output;
};

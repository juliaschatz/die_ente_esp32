#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"

#define INCR_ENUM(val, type) val = static_cast<type>(static_cast<int>(val) + 1); 

mcpwm_unit_t Motor::next_unit = MCPWM_UNIT_0;
mcpwm_io_signals_t Motor::next_output = MCPWM0A;

void Motor::initUnit(mcpwm_unit_t unit, mcpwm_timer_t timer) {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit, timer, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

Motor::Motor(int gpio_pwm) {
    this->gpio_pwm = gpio_pwm;
}

void Motor::begin() {
  if (Motor::next_unit >= MCPWM_UNIT_MAX) {
    // todo throw error
    return;
  }
  // Each unit has 6 outputs, indexed by the timer (0,1,2) then operator (A,B)
  mcpwm_unit_t my_unit = Motor::next_unit;
  mcpwm_timer_t my_timer = (mcpwm_timer_t) (Motor::next_output / 2);
  mcpwm_io_signals_t my_operator = (mcpwm_io_signals_t) (Motor::next_output % 2);
  if (my_operator == 0) {
    // First time using this timer, need to initialize it
    Motor::initUnit(my_unit, my_timer);
  }
  mcpwm_gpio_init(Motor::next_unit, Motor::next_output, this->gpio_pwm);
  INCR_ENUM(Motor::next_output, mcpwm_io_signals_t);
  if (Motor::next_output > MCPWM2B) {
    INCR_ENUM(Motor::unit, mcpwm_unit_t);
    Motor::next_output = MCPWM0A;
  }

  this->unit = my_unit;
  this->timer = my_timer;
  this->op = my_operator;
}

/**
 * Sets the speed between 0 and 1
 */
void Motor::setSpeed(float speed) {
    // Clamp magnitude
    speed = speed > MAX_POWER ? MAX_POWER : speed;
    speed = speed < 0.01 ? 0.0 : speed;
    // Interpolate magnitude to us
    int micros;
    if (speed > 0.0) {
        int range = MAX_PULSE_US - MIN_PULSE_US;

        micros = MIN_PULSE_US + speed * range;
    }
    else {
        micros = OFF_PULSE_US;
    }
    mcpwm_set_duty_in_us(this->unit, this->timer, (mcpwm_generator_t) this->op, micros);
}
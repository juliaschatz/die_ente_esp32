#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "driver/uart.h"
#include "ahrs.h"
#include "network.h"
#include "math.h"
#include "gnc.h"
#include <string.h>

using namespace Eigen;

Motor fr(19);
Motor fl(18);
Motor rr(5);
Motor rl(17);
Adafruit_BNO055 ahrs(UART_NUM_2);
extern QueueHandle_t xNetQueue;

const double Kl = 0.9968/4; // Throttle lift constant. Per motor required to maintain altitude

void setupMotors() {
  fr.begin();
  fl.begin();
  rl.begin();
  rr.begin();
}

void modifyThrottle(Vector4d& u, double throttle) {
  // Modify the control input so that the minimum is zero
  double min = fmin(u(0), fmin(u(1), fmin(u(2), u(3))));
  if (min < 0) {
    u += -1. * min * Vector4d(1.0, 1.0, 1.0, 1.0);
  }
  // Scale control inputs so that they remain sane relative to the applied throttle
  double max = fmax(u(0), fmax(u(1), fmax(u(2), u(3))));
  u *= throttle / max;
}

void updateState(Vector3d& rates, Quaterniond& attitude) {
  // Get state from IMU
  attitude = ahrs.getQuat();
  rates = ahrs.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GYROSCOPE);
}

bool setupAHRS() {
  if (ahrs.begin(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF, GPIO_NUM_23, GPIO_NUM_22)) {
    // any addl setup needed here? axis remap?
    return true;
  }
  else {
    return false;
  }
}

void flightControllerTask(void* arg) {
  setupController();
  setupMotors();

  // Arming/calibration sequence
  fr.setSpeed(0.0);
  fl.setSpeed(0.0);
  rl.setSpeed(0.0);
  rr.setSpeed(0.0);
  vTaskDelay(200);
  fr.setSpeed(1.0);
  fl.setSpeed(1.0);
  rl.setSpeed(1.0);
  rr.setSpeed(1.0);
  vTaskDelay(200);
  fr.setSpeed(0.0);
  fl.setSpeed(0.0);
  rl.setSpeed(0.0);
  rr.setSpeed(0.0);

  if (!setupAHRS()) {
    printf("AHRS setup failed\n");
    return;
  }
  TickType_t lastTime = xTaskGetTickCount();
  Packet lastControlState = {.timestamp = lastTime};
  Quaterniond attitude;
  Quaterniond desired_attitude(1.0,0.0,0.0,0.0);
  Vector3d rates;
  double desired_yaw = 0.0;
  while (1) {
    updateState(rates, attitude);

    // Get reference state from controls
    xQueueReceive(xNetQueue, &lastControlState, 0);

    /*desired_state << 0,
                      15.0 * lastControlState.commandPitch / 127.0,
                      15.0 * lastControlState.commandRoll / 127.0,
                      0.,0.,0.;*/

    // Calculate control effort
    Vector3d desired_angvel = attitude_controller(attitude, desired_attitude);
    Vector4d u = angvel_controller(rates, desired_angvel);

    // Apply throttle
    double throttle = Kl + lastControlState.commandThrottle * 0.25 / 127.0;
    //modifyThrottle(u, throttle);
    u += Vector4d(1.0, 1.0, 1.0, 1.0) * throttle;
    printf("%f %f %f %f : %f %f %f %f : %f %f %f\n", u(0), u(1), u(2), u(3), attitude.w(), attitude.x(), attitude.y(), attitude.z(), rates(0), rates(1), rates(2));
    if (lastTime - lastControlState.timestamp > CONTROL_TIMEOUT || lastControlState.commandThrottle < 0.1) {
        u *= 0.0;
    }
    fr.setSpeed(u(0));
    fl.setSpeed(u(1));
    rl.setSpeed(u(2));
    rr.setSpeed(u(3));

    delayUntil(&lastTime, 10);
  }
}
#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "driver/uart.h"
#include "ahrs.h"
#include "network.h"
#include "math.h"
#include <string.h>

Motor fr(19);
Motor fl(18);
Motor rr(5);
Motor rl(17);
Adafruit_BNO055 ahrs(UART_NUM_2);
extern QueueHandle_t xNetQueue;

Eigen::Matrix<double, 4, 6> K;
const float Kl = 0.9968/4; // Throttle lift constant. Per motor required to maintain altitude

void setupMotors() {
  fr.begin();
  fl.begin();
  rl.begin();
  rr.begin();
}

void setupController() {
   K <<
   0.012936, -0.010663, -0.016771,  0.001856, -0.001532, -0.001287, 
	-0.012936, -0.010663,  0.016771, -0.001856, -0.001532,  0.001287, 
	 0.012936,  0.010663,  0.016771,  0.001856,  0.001532,  0.001287, 
	-0.012936,  0.010663, -0.016771, -0.001856,  0.001532, -0.001287;
}

void modifyThrottle(Eigen::Vector4d& u, float throttle) {
  // Modify the control input so that the minimum is zero
  float min = fmin(u(0), fmin(u(1), fmin(u(2), u(3))));
  if (min < 0) {
    u += -1. * min * Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  }
  // Scale control inputs so that they remain sane relative to the applied throttle
  float max = fmax(u(0), fmax(u(1), fmax(u(2), u(3))));
  u *= throttle / max;
}

void updateState(Eigen::Vector<double, 6>& state) {
    // Get state from IMU
    Eigen::Vector3d euler = ahrs.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_EULER);
    Eigen::Vector3d rates = ahrs.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_GYROSCOPE);
    // Make sure we got good gyro data
    if (!(rates(0) == 0 && rates(1) == 0 && rates(2) == 0)) {
        // Reorder gyro data (comes out rpy, needs to be ypr)
        float dyaw = rates(2);
        float droll = rates(0);
        rates(2) = -droll;
        rates(1) *= -1.0;
        rates(0) = dyaw;
    }
    // Make sure we got good euler data
    if (!(euler(0) == 0 && euler(1) == 0 && euler(2) == 0)) {
        // Change yaw to be (-180, 180]
        if (euler(0) > 180.0) {
            euler(0) -= 360.0;
        }
        // Fix orientation (Follow RHR)
        euler(0) *= -1;
    }
    state.head(3) = euler;
    state.tail(3) = rates;
}

bool setupAHRS() {
    return ahrs.begin(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF, GPIO_NUM_23, GPIO_NUM_22);
}

void flightControllerTask(void* arg) {
  setupController();
    setupMotors();
    if (!setupAHRS()) {
        printf("AHRS setup failed\n");
    }
    TickType_t lastTime = xTaskGetTickCount();
    Packet lastControlState = {.timestamp = lastTime};
    Eigen::Vector<double, 6> state;
    Eigen::Vector<double, 6> desired_state;
    float desired_yaw = 0;
    while (1) {
        updateState(state);

        // Get reference state from controls
        xQueueReceive(xNetQueue, &lastControlState, 0);
        desired_yaw -= lastControlState.commandYaw * 0.9 / 127.0;
        if (desired_yaw > 180.0) {
            desired_yaw -= 360.0;
        }
        if (desired_yaw <= -180.0) {
            desired_yaw += 360.0;
        }
        // Fix state yaw so that r-x is always in the desired range
        // Fixes yaw discontinuity
        float state_yaw = desired_yaw - state(0);
        if (fabs(state_yaw) > 180) {
            if (state_yaw < 0) {
                state_yaw = 360 + state_yaw;
            }
            else {
                state_yaw = -360 + state_yaw;
            }
        }
        state(0) = 0;//-state_yaw;

        desired_state << 0,
                          15.0 * lastControlState.commandPitch / 127.0,
                          15.0 * lastControlState.commandRoll / 127.0,
                          0.,0.,0.;

        // Calculate control effort
        Eigen::Vector4d u = K*(desired_state - state);

        // Apply throttle
        float throttle = Kl + lastControlState.commandThrottle * 0.25 / 127.0;
        modifyThrottle(u, throttle);
        if (lastTime - lastControlState.timestamp > CONTROL_TIMEOUT || lastControlState.commandThrottle < 0.1) {
            u *= 0.0;
        }
        fr.setSpeed(u(0));
        fl.setSpeed(u(1));
        rl.setSpeed(u(2));
        rr.setSpeed(u(3));
        printf("%f %f %f %f %f %f\n", state(0), state(1), state(2), state(3), state(4), state(5));

        delayUntil(&lastTime, 10);
    }
}
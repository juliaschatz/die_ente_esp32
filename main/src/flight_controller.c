#include "flight_controller.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_util.h"
#include "driver/uart.h"
#include "ahrs.h"
#include "linear.h"
#include "network.h"
#include "math.h"
#include <string.h>

Motor* fr;
Motor* fl;
Motor* rr;
Motor* rl;
Adafruit_BNO055 * ahrs;
extern QueueHandle_t xNetQueue;

float K_data[] = {
   0.012936, -0.010663, -0.016771,  0.001856, -0.001532, -0.001287, 
	-0.012936, -0.010663,  0.016771, -0.001856, -0.001532,  0.001287, 
	 0.012936,  0.010663,  0.016771,  0.001856,  0.001532,  0.001287, 
	-0.012936,  0.010663, -0.016771, -0.001856,  0.001532, -0.001287
};
const float Kl = 0.9968/4; // Throttle lift constant. Per motor required to maintain altitude

Matrix K = {.data = K_data, .rows = 4, .cols = 6};

void setupMotors() {
    fr = createMotor(19);
    fl = createMotor(18);
    rl = createMotor(5);
    rr = createMotor(17);
}

void applyThrottle(Vector* u, float throttle) {
    float min = fmin(u->data[0], fmin(u->data[1], fmin(u->data[2], u->data[3])));
    if (min < 0) {
        vectorAddScalar(u, -1. * min);
    }

    float max = fmax(u->data[0], fmax(u->data[1], fmax(u->data[2], u->data[3])));
    vectorScale(u, throttle / max);
}

void updateState(float* state_data) {
    // Get state from IMU
    float temp_state[6];
    getVector(ahrs, VECTOR_EULER, temp_state);
    getVector(ahrs, VECTOR_GYROSCOPE, temp_state+3);
    // Make sure we got good gyro data
    if (!(temp_state[3] == 0 && temp_state[4] == 0 && temp_state[5] == 0)) {
        // Reorder gyro data (comes out rpy, needs to be ypr)
        float dyaw = temp_state[5];
        float droll = temp_state[3];
        temp_state[5] = -droll;
        temp_state[4] = -1. * temp_state[4];
        temp_state[3] = dyaw;
        memcpy(state_data+3, temp_state+3, 3*sizeof(float));
    }
    // Make sure we got good euler data
    if (!(temp_state[0] == 0 && temp_state[1] == 0 && temp_state[2] == 0)) {
        memcpy(state_data, temp_state, 3*sizeof(float));
        // Change yaw to be (-180, 180]
        if (state_data[0] > 180.0) {
            state_data[0] -= 360.0;
        }
        // Fix orientation (Follow RHR)
        state_data[0] *= -1;
    }
}

int setupAHRS() {
    ahrs = createBNO055(UART_NUM_2);
    return ahrsBegin(ahrs, 23, 22, OPERATION_MODE_NDOF);
}

void flightControllerTask(void* arg) {
    setupMotors();
    if (!setupAHRS()) {
        printf("AHRS setup failed\n");
    }
    TickType_t lastTime = xTaskGetTickCount();
    Packet lastControlState = {.timestamp = lastTime};
    float state[6];
    Vector x = {.data = state, .length = 6};
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
        float state_yaw = desired_yaw - state[0];
        if (fabs(state_yaw) > 180) {
            if (state_yaw < 0) {
                state_yaw = 360 + state_yaw;
            }
            else {
                state_yaw = -360 + state_yaw;
            }
        }
        state[0] = 0;//-state_yaw;

        float r_data[] = {0,
                          15.0 * lastControlState.commandPitch / 127.0,
                          15.0 * lastControlState.commandRoll / 127.0,
                          0.,0.,0.};
        Vector r = {.data = r_data, .length = 6};

        // Apply control effort
        Vector dr = vectorSub(r, x);
        Vector u = matVecMul(K, dr);

        // Apply throttle
        float throttle = Kl + lastControlState.commandThrottle * 0.25 / 127.0;
        applyThrottle(&u, throttle);
        if (lastTime - lastControlState.timestamp > CONTROL_TIMEOUT || lastControlState.commandThrottle < 0.1) {
            vectorScale(&u, 0);
        }
        printVec(&u);
        setMotorSpeed(fr, u.data[0]);
        setMotorSpeed(fl, u.data[1]);
        setMotorSpeed(rl, u.data[2]);
        setMotorSpeed(rr, u.data[3]);

        free(u.data);
        free(dr.data);
        delayUntil(&lastTime, 10);
    }
}
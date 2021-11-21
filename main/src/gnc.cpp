#include "gnc.h"

Matrix3d angvelGainMat;
Matrix<double, 4, 3> steeringMatrix;

const double yaw_angvel_kp = 0.0001856;
const double pitch_angvel_kp = 0.0001287;
const double roll_angvel_kp = 0.0002587;
const double time_constant = 0.001;

void setupController() {
  steeringMatrix <<
    -1, -1, -1,
     1, -1,  1,
     1,  1, -1,
    -1,  1,  1;
  angvelGainMat = (Vector3d() << yaw_angvel_kp, pitch_angvel_kp, roll_angvel_kp).finished().asDiagonal();
}

Vector4d angvel_controller(Vector3d angvel_estimate, Vector3d angvel_target) {
  return steeringMatrix * angvelGainMat * (angvel_target - angvel_estimate);
}

Vector3d attitude_controller(Quaterniond attitude_estimate, Quaterniond attitude_target) {
  // implements https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
  Quaterniond qe = attitude_estimate.inverse() * attitude_target;
  return (2 / time_constant) * sgn(qe.w()) * qe.vec();
}

double sgn(double x) {
  return x >= 0.0 ? 1.0 : -1.0;
}
#pragma once

#include <Eigen>
using namespace Eigen;


void setupController();
Vector4d angvel_controller(Vector3d angvel_estimate, Vector3d angvel_target);
Vector3d attitude_controller(Quaterniond attitude_estimate, Quaterniond attitude_target);
double sgn(double x);
#include "control_drivers.h"

const double riderKp = 9;
const double riderKi = 0;
const double riderKd = 12.5;
const double riderSetpoint = 0;

const double noRiderKp = 9;
const double noRiderKi = 0.15;
const double noRiderKd = 15;
const double noRiderSetpoint = 7.4;

void handleChangingRiderMode(
  bool* previousRiderMode, 
  bool riderMode, 
  double* integral,
  double* targetSetpoint,
  double* targetKp,
  double* kd,
  double* ki
) {
  if (*previousRiderMode != riderMode) {  //gets activated when you switch rider mode AND FIRST TIME BOOT UP
      *previousRiderMode = riderMode;
      *integral = 0;
      if (riderMode) {
        *targetSetpoint = riderSetpoint;
        *targetKp = riderKp;
        *kd = riderKd;
        *ki = riderKi;
      } else {
        *targetSetpoint = noRiderSetpoint;
        *targetKp = noRiderKp;
        *kd = noRiderKd;
        *ki = noRiderKi;
      }  //if rider is not present
    }
}
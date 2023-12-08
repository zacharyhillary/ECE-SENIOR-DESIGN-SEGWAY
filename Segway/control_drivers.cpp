#include "control_drivers.h"
#include "array_utils.h"
#include "Arduino.h"

const double riderKp = 9;
const double riderKi = 0;
const double riderKd = 12.5;
const double riderSetpoint = 0;

const double noRiderKp = 9;
const double noRiderKi = 0.15;
const double noRiderKd = 12.5;
const double noRiderSetpoint = 7.2;

const double boundedAngle = 3;
const double boundedRiderKp = 10;
const double boundedRiderKi = 0.12;
const double boundedRiderKd = 12.5;

void handleChangingRiderMode(
  bool* previousRiderMode, 
  bool riderMode, 
  bool* resetConfig,
  double* integral,
  double* targetSetpoint,
  double* targetKp,
  double* kd,
  double* ki
) {
    if (*previousRiderMode != riderMode || *resetConfig) {  //gets activated when you switch rider mode AND FIRST TIME BOOT UP
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
        *resetConfig = false;
    }
}

void handleRiderModeConditions(
  bool riderMode,
  double currentAngle,
  double* targetKp,
  double* ki,
  double* kd,
  double* integral) {
  if (riderMode) {
    // NECESSARY RIDERMODE CHECK

    // bounded angle operations
    if (abs(currentAngle) < boundedAngle) {  //increase Ki past certain angle thresholds to keep in bounded angles
      *targetKp = boundedRiderKp;
      *ki = boundedRiderKi;
      *kd = boundedRiderKd;
    } else {
      *integral *= 0.98;
      *targetKp = riderKp;
      *ki = riderKi;
      *kd = riderKd;
    }



    // END OF NECESSARY RIDERMODE CHECK
  }
}

int OUTPUT_ARRAY_SIZE = 10;
long OVERRIDE_DELAY = 1000;
long COOLDOWN_TIME = 1000;
const double ANGLE_CHANGE = 2;
long overrideTimestamp = millis();
bool overriding = false;
int lastOutputs[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void handleRiderlessModeConditions(
  bool riderMode,
  int output,
  bool* resetConfig,
  double* targetSetpoint,
  double* kp
) {
  if (!riderMode) {
    // NECESSARY RIDERMODE CHECK

    // Overriding setpoint
    if (overriding) {
      if (millis() - overrideTimestamp > OVERRIDE_DELAY) {
        overriding = false;
        *resetConfig = true;
        clearIntArray(lastOutputs, OUTPUT_ARRAY_SIZE);
        overrideTimestamp = millis();
      }
    } else {
      pushToArray(lastOutputs, output, OUTPUT_ARRAY_SIZE);
      double average = avgOfIntArray(lastOutputs, OUTPUT_ARRAY_SIZE);
      if (millis() - overrideTimestamp > COOLDOWN_TIME) {
        if (average > 10 && checkEachGreaterThan(lastOutputs, 10, OUTPUT_ARRAY_SIZE)) {
          *targetSetpoint = *targetSetpoint + ANGLE_CHANGE;
          *kp = *kp + 3;
          overriding = true;
          overrideTimestamp = millis();
        } else if (average < -10 && checkEachLessThan(lastOutputs, -10, OUTPUT_ARRAY_SIZE)) {
          *targetSetpoint = *targetSetpoint - ANGLE_CHANGE;
          *kp = *kp + 3;
          overriding = true;
          overrideTimestamp = millis();
        }
      }
    }



    // END OF NECESSARY RIDERMODE CHECK
  }
}
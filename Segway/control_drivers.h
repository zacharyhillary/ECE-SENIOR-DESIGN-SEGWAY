#ifndef CONTROL_DRIVERS_H
#define CONTROL_DRIVERS_H

extern const double riderKp;
extern const double riderKi;
extern const double riderKd;
extern const double riderSetpoint;

extern const double noRiderKp;
extern const double noRiderKi;
extern const double noRiderKd;
extern const double noRiderSetpoint;

extern const double boundedRiderKp;
extern const double boundedRiderKi;
extern const double boundedRiderKd;

void handleChangingRiderMode(
  bool* previousRiderMode, 
  bool riderMode, 
  bool* resetConfig,
  double* integral,
  double* targetSetpoint,
  double* targetKp,
  double* kd,
  double* ki
);

void handleRiderModeConditions(
  bool riderMode,
  double currentAngle,
  double* targetKp,
  double* ki,
  double* kd,
  double* integral
);

void handleRiderlessModeConditions(
  bool riderMode,
  int output,
  bool* resetConfig,
  double* targetSetpoint,
  double* kp
);

#endif /* CONTROL_DRIVERS_H */
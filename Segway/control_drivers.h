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

void handleChangingRiderMode(
  bool* previousRiderMode, 
  bool riderMode, 
  double* integral,
  double* targetSetpoint,
  double* targetKp,
  double* kd,
  double* ki
);

#endif /* CONTROL_DRIVERS_H */
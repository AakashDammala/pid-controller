#pragma once

class PIDController {
public:
  PIDController();
  ~PIDController();

  double compute(double error);

private:
  // PID gains
  double Kp_;
  double Ki_;
  double Kd_;
};

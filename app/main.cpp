#include "libpid.hpp"
#include <iostream>

int main() {
  // Constructing PIDController object
  // Kp, Ki, Kd, delta_time (s), min_output, max_output
  PIDController pid_controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);

  // Setpoint and a few measurements
  double setpoint = 5.0;
  double measurements[] = {0.0, 1.5, 3.0, 4.2, 4.8, 5.0, 5.1};

  for (double m : measurements) {
    double out = pid_controller.compute(setpoint, m);
    std::cout << "setpoint=" << setpoint << " measurement=" << m
              << " -> output=" << out << '\n';
  }

  return 0;
}

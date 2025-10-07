// #include "lib.hpp"
#include "libpid.hpp"
#include <iostream>

int main() {
  PIDController pid_controller;
  double error = 10.0;
  double response = pid_controller.compute(error);
  printf("PID controller response for error [%.2lf] is [%.2lf]\n", error, response);
  return 0;
}

/**
 * @file libpid.hpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Simple PID controller interface.
 * @version 0.1
 * @date 2025-10-07
 * @copyright Copyright (c) 2025
 */

#pragma once

/**
 * @class PIDController
 * @brief Minimal PID controller for scalar inputs.
 */
class PIDController {
public:
  /**
   * @brief Construct the PID controller.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param delta_time Control timestep in seconds.
   * @param min_output Minimum output (clamp lower bound).
   * @param max_output Maximum output (clamp upper bound).
   */
  PIDController(const double Kp, const double Ki, const double Kd,
                const double delta_time, const double min_output,
                const double max_output);

  /**
   * @brief Destroy the PIDController.
   */
  ~PIDController();

  /**
   * @brief Compute controller output for a single timestep.
   * @param target_setpoint Desired setpoint.
   * @param measured_value Current measurement.
   * @return double Controller output clamped to [min_output, max_output].
   */
  double compute(double target_setpoint, double measured_value);

private:
  const double Kp_; // proportional gain
  const double Ki_; // integral gain
  const double Kd_; // differential gain
  const double delta_time_; // loop time of the controller
  const double min_output_; // minimum output provided by the controller
  const double max_output_; // maximum output provided by the controller 
};

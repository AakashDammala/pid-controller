/**
 * @file test.cpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Unit tests for the PIDController.
 * @version 0.1
 * @date 2025-10-07
 * @copyright Copyright (c) 2025
 *
 * PID Controller Formulas:
 * Error = Setpoint - Measured Value
 * Proportional Output = Kp * Error
 * Integral Output = Ki * (Previous Integral Error + Error)
 * Derivative Output = Kd * (Error - Previous Error) / dt
 * Total Output = Proportional Output + Integral Output + Derivative Output
 */

#include <gtest/gtest.h>

#include "libpid.hpp"

/**
 * @brief When setpoint equals measurement, controller should return zero value.
 * PID Gains: Kp = 1.0   Ki = 0.1   Kd = 0.01
 * delta_time = 0.1   min_output = -100.0   max_output = 100.0
 * setpoint = measurement = 5.0
 * error = setpoint - measurement = 5.0 - 5.0 = 0.0
 * Pout = 1.0 * 0.0 = 0.0
 * integral = 0.0 * 0.1 = 0.0
 * Iout = 0.1 * 0.0 = 0.0
 * derivative = (0.0 - 0.0) / 0.1 = 0.0
 * Dout = 0.01 * 0.0 = 0.0
 * expected_output = 0.0
 */
TEST(PIDControllerTest, SetpointEqualsMeasurement) {
  // Construct controller with example gains and limits.
  PIDController controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);

  double setpoint = 5.0;
  double measurement = 5.0;  // same as setpoint
  double expected = 0.0;

  double output = controller.compute(setpoint, measurement);

  // Expect the controller output to be zero since error is zero
  // Use a small tolerance in case of floating-point computations.
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Proportional only (Ki, Kd = 0) control test
 * PID Gains: Kp = 2.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * error = setpoint - meassurement = 3.0 - 1.0 = 2.0
 * Pout = 2.0 * 2.0 = 4.0
 * Iout = 0.0
 * Dout = 0.0
 * expected_output = 4.0
 * expected_output within clamp limits, so no clamps applied
 */
TEST(PIDControllerTest, ProportionalOnlyBehavior) {
  const double Kp = 2.0;
  // Ki=0 and Kd=0 to test proportional-only response
  PIDController controller(Kp, 0.0, 0.0, 0.1, -10.0, 10.0);

  double setpoint = 3.0;
  double measurement = 1.0;
  double expected = 4.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to max_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = 10.0 - 0.0 = 10.0
 * Pout = 100.0 * 10.0 = 1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = 1000.0
 * expected_output = clamp(1000.0) = 5.0
 */
TEST(PIDControllerTest, ClampsToMaxOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = 10.0;
  double measurement = 0.0;
  double expected = 5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to min_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = -10.0 - 0.0 = -10.0
 * Pout = 100.0 * -10.0 = -1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = -1000.0
 * expected_output = clamp(-1000.0) = -5.0
 */
TEST(PIDControllerTest, ClampsToMinOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = -10.0;
  double measurement = 0.0;
  double expected = -5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Integral only (Kp, Kd = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.5   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error = setpoint - measurement = 5.0 - 3.0 = 2.0
 * integral = prev_integral + (error * delta_time) = 0.0 + (2.0 * 0.1) = 0.2
 * expected_output1 = Ki * integral = 0.5 * 0.2 = 0.1
 *
 * Second step:
 * integral = previous_integral + (error * delta_time) = 0.2 + (2.0 * 0.1) = 0.4
 * expected_output2 = Ki * integral = 0.5 * 0.4 = 0.2
 */
TEST(PIDControllerTest, IntegralOnlyBehavior) {
  const double Ki = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, Ki, 0.0, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement = 3.0;
  double expected1 = 0.1;

  // First step
  double output1 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step, integral should accumulate
  double expected2 = 0.2;
  double output2 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Differential only (Kp, Ki = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.0   Kd = 0.5
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error1 = setpoint - measurement1 = 5.0 - 3.0 = 2.0
 * previous_error = 0.0 (initial)
 * derivative = (error1 - previous_error) / dt = (2.0 - 0.0) / 0.1 = 20.0
 * expected_output1 = Kd * derivative = 0.5 * 20.0 = 10.0
 *
 * Second step:
 * error2 = setpoint - measurement2 = 5.0 - 4.0 = 1.0
 * derivative = (error2 - error1) / dt = (1.0 - 2.0) / 0.1 = -10.0
 * expected_output2 = Kd * derivative = 0.5 * (-10.0) = -5.0
 */
TEST(PIDControllerTest, DifferentialOnlyBehavior) {
  const double Kd = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, 0.0, Kd, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement1 = 3.0;
  double expected1 = 10.0;

  // First step
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step
  double measurement2 = 4.0;
  double expected2 = -5.0;
  double output2 = controller.compute(setpoint, measurement2);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Test with all PID terms active over two iterations
 * PID Gains: Kp = 1.0   Ki = 0.5   Kd = 0.1
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * setpoint = 10.0
 *
 * First iteration (measurement = 5.0):
 * error1 = 10.0 - 5.0 = 5.0
 * Pout1 = 1.0 * 5.0 = 5.0
 * integral1 = 0.0 + 5.0 * 0.1 = 0.5
 * Iout1 = 0.5 * 0.5 = 0.25
 * derivative1 = (5.0 - 0.0) / 0.1 = 50.0
 * Dout1 = 0.1 * 50.0 = 5.0
 * output1 = 5.0 + 0.25 + 5.0 = 10.25
 * clamped_output1 = 10.0
 *
 * Second iteration (measurement = 7.0):
 * error2 = 10.0 - 7.0 = 3.0
 * Pout2 = 1.0 * 3.0 = 3.0
 * integral2 = 0.5 + 3.0 * 0.1 = 0.8
 * Iout2 = 0.5 * 0.8 = 0.4
 * derivative2 = (3.0 - 5.0) / 0.1 = -20.0
 * Dout2 = 0.1 * -20.0 = -2.0
 * output2 = 3.0 + 0.4 - 2.0 = 1.4
 */
TEST(PIDControllerTest, CombinedPIDBehavior) {
  const double Kp = 1.0;
  const double Ki = 0.5;
  const double Kd = 0.1;
  const double dt = 0.1;
  PIDController controller(Kp, Ki, Kd, dt, -10.0, 10.0);

  double setpoint = 10.0;

  // First iteration
  double measurement1 = 5.0;
  double expected1 = 10.0;  // Would be 10.25 but clamped to 10.0
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second iteration
  double measurement2 = 7.0;
  double expected2 = 1.4;
  double output2 = controller.compute(setpoint, measurement2);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

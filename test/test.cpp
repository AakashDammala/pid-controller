/**
 * @file test.cpp
 * @author Aakash Shetty Dammala (asd@umd.edu)
 * @brief Unit tests for the PIDController.
 * @version 0.1
 * @date 2025-10-07
 * @copyright Copyright (c) 2025
 */

#include "libpid.hpp"
#include <gtest/gtest.h>

/**
 * @brief When setpoint equals measurement, controller should return same value.
 */
TEST(PIDControllerTest, ReturnsSameWhenSetpointEqualsMeasurement) {
  // Construct controller with example gains and limits.
  PIDController controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);

  double setpoint = 5.0;
  double measurement = 5.0; // same as setpoint

  double output = controller.compute(setpoint, measurement);

  // Expect the controller output to equal the setpoint when already at setpoint.
  // Use a small tolerance in case of floating-point computations.
  EXPECT_NEAR(output, setpoint, 1e-6);
}

/**
 * @brief With Ki=Kd=0, controller should behave like a proportional controller.
 */
TEST(PIDControllerTest, ProportionalOnlyBehavior) {
  const double Kp = 2.0;
  // Ki=0 and Kd=0 to test proportional-only response
  PIDController controller(Kp, 0.0, 0.0, 0.1, -10.0, 10.0);

  double setpoint = 3.0;
  double measurement = 1.0;

  double expected = Kp * (setpoint - measurement);
  // Expected should be within clamp range
  if (expected > 10.0) expected = 10.0;
  if (expected < -10.0) expected = -10.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to min/max output.
 */
TEST(PIDControllerTest, ClampsToOutputLimits) {
  // Use a large Kp to force a value beyond the clamp limits
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = 10.0;
  double measurement = 0.0;

  double output = controller.compute(setpoint, measurement);
  // Expected raw output = Kp*(10 - 0) = 1000 -> clamped to 5.0
  EXPECT_NEAR(output, 5.0, 1e-6);

  // Also test lower clamp
  double setpoint2 = -10.0;
  double measurement2 = 0.0;
  double output2 = controller.compute(setpoint2, measurement2);
  // Expected raw = 100 * (-10) = -1000 -> clamped to -5.0
  EXPECT_NEAR(output2, -5.0, 1e-6);
}


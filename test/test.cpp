#include <gtest/gtest.h>
#include "libpid.hpp"

TEST(dummy_test, this_should_pass) {
  PIDController controller;
  double error = 10.0;
  double controller_output = controller.compute(10.0);
  EXPECT_EQ(error, controller_output);
}

TEST(dummy_test, this_will_fail) {
  PIDController controller;
  double error = 10.0;
  double controller_output = controller.compute(10.0);
  EXPECT_EQ(error, controller_output * 2);
}

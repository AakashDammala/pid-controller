# PID Controller

![CICD Workflow status](https://github.com/AakashDammala/pid-controller/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/AakashDammala/pid-controller/branch/main/graph/badge.svg)](https://codecov.io/gh/AakashDammala/pid-controller) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


## Overview

This project is built as part of ENPM-700 Software Development for Robotics course. 
Here, we develop a practical intuition for pair programming, unit testing and using CI/CD pipelines for robotics projects through building a simple pid controller C++ library.

In the first part of the project
1. Aakash (asd@umd.edu) - Acts as 'driver'
2. Tirth (tsadaria@umd.edu) - Acts as 'navigator'

In the second part of the project
1. Siddhant (iamsid@umd.edu) - Acts as 'driver'
2. Grayson (gglibert@umd.edu) - Acts as 'navigator'


## Design Review Discussion

The `PIDController` class is a minimal PID controller for scalar inputs, providing a well-structured design:

* Clear class interface with public `compute()` method
* Private helper methods for modular calculation of P, I, and D components
* Appropriate use of member variables to track state (`prev_error_`, `integral_error_`)
* Comprehensive test suite covering all edge cases

## Implementation Approach

Following TDD principles, the PID controller was implemented in logical commits:

* Implemented helper methods: Created the three component calculation methods (proportional, integral, derivative)
* Completed `compute()` method: Integrated all components with error calculation and output clamping
* Added input validation: Proper handling of timestep and internal state updates

## Key Implementation Details

* Proportional component: `Pout = Kp_ * error`
* Integral component: `integral_error_ += error * delta_time_; Iout = Ki_ * integral_error_`
* Derivative component: `Dout = Kd_ * (error - prev_error_) / delta_time_`
* Output clamping: Ensures output stays within `[min_output_, max_output_]`
* State tracking: Uses `prev_error_` and `integral_error_` for derivative and integral calculations

## Test Results

All unit tests pass successfully:

* ✅ Setpoint equals measurement test
* ✅ Proportional only behavior test
* ✅ Clamps to max output test
* ✅ Clamps to min output test
* ✅ Integral only behavior test
* ✅ Differential only behavior test
* ✅ Combined PID behavior test

## Code Quality

* Documentation: All methods documented with Doxygen-style comments
* Robustness: Correct accumulation of integral term and derivative computation
* Clamping: Output consistently respects specified bounds


## Instructions to run the project

```bash
# Download the code:
  git clone https://github.com/AakashDammala/pid-controller.git
  cd pid-controller

# Configure the project
  cmake -S ./ -B build/

# Build the project
  cmake --build build/

# Run the program
  ./build/app/shell-app

# Run unit tests
  ./build/test/cpp-test

# Generate and view the code coverage report offline
  # Configure CMake to generate coverage report 
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
  # build the project
  cmake --build build/ --clean-first --target all test_coverage
  # View the coverage report
  open build/test_coverage/index.html

# Generate and view doxygen documentation
  cmake --build build/ --target docs
  open docs/html/index.html

```

## Building for code coverage:

```bash
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.
```

You can also get code coverage report for the pid_controller_example target, instead of unit test. Repeat the previous 2 steps but with the *app_coverage* target:

``` bash
# Now, do another clean compile, run pid_controller_example, and generate its covereage report
  cmake --build build/ --clean-first --target all app_coverage
# open a web browser to browse the test coverage report
  open build/app_coverage/index.html

This generates a index.html page in the build/app_coverage sub-directory that can be viewed locally in a web browser.
```

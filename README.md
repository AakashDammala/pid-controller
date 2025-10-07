# PID Controller

![CICD Workflow status](https://github.com/AakashDammala/pid-controller/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/AakashDammala/pid-controller/branch/pid-controller-impl/graph/badge.svg)](https://codecov.io/gh/AakashDammala/pid-controller) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


## Overview

This project is built as part of ENPM-700 Software Development for Robotics course. 
Here, we develop a practical intuition for pair programming, unit testing and using CI/CD pipelines for robotics projects through building a simple pid controller C++ library.

In the first part of the project
1. Aakash (asd@umd.edu) - Acts as 'driver'
2. Tirth (tsadaria@umd.edu) - Acts as 'navigator'

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

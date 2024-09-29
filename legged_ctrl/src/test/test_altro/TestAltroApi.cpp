//
// Created by Brian Jackson on 9/12/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "fmt/core.h"

#include "altro/altro.hpp"

int main() {
  fmt::print("Hello from testing suite!\n");

  int N = 10;
  altro::ALTROSolver solver(N);

  return 0;
}

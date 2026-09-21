// SPDX-License-Identifier: AGPL-3.0-only
// Tiny cross-TU API so matrix_runner can drive the safe_unique fixture
// before and after migration (the fixture classes are TU-local on purpose —
// the matrix analyses one TU at a time).
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

std::shared_ptr<rclcpp::Node> matrix_make_safe_unique();
void matrix_tick_safe_unique(rclcpp::Node *n);

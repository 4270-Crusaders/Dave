#pragma once

#include <cstdint>

namespace example_position_arm_constants {

inline constexpr std::uint8_t kMotorPort = 3;
inline constexpr double kKp = 2.2;
inline constexpr double kKi = 0.0;
inline constexpr double kKd = 0.15;
inline constexpr double kIntegralWindup = 5000.0;
inline constexpr double kAngleToleranceDeg = 2.0;
inline constexpr std::int32_t kMaxVoltageMv = 10000;

} // namespace example_position_arm_constants

#pragma once

#include "pros/motors.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace example_roller_constants {

/** Port + gear for each motor in the group (1, 2, or more). Signed port = reversed. */
struct MotorSlot {
	std::int8_t port;
	pros::v5::MotorGears gear;
};

/** Default template: two blue 600 RPM cartridges; trim or extend the array for your robot. */
inline constexpr std::array<MotorSlot, 2> kMotorSlots = {{
	{1, pros::v5::MotorGears::blue},
	{-2, pros::v5::MotorGears::blue},
}};

/** Velocity PID: error = (target_rpm − average actual RPM); output adds to feedforward (millivolts). */
inline constexpr double kVelocityKp = 18.0;
inline constexpr double kVelocityKi = 0.0;
inline constexpr double kVelocityKd = 0.35;
inline constexpr double kVelocityIntegralWindup = 5000.0;

/** Rough feedforward: millivolts per RPM toward the target (tune with PID at rest). */
inline constexpr double kVelocityFfMvPerRpm = 4.5;

/** |pct| = 1 maps to this many RPM (should match cartridge + wheel choice). */
inline constexpr double kMaxAbsRpm = 600.0;

inline constexpr std::int32_t kMaxVoltageMv = 12000;

} // namespace example_roller_constants

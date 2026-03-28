#pragma once

#include "subsystems/example_roller/ExampleVelocityRollerConstants.h"
#include "utils/command/runCommand.h"
#include "utils/command/subsystem.h"
#include "utils/control/PidController.h"
#include "pros/motors.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace example_roller {

/** Build `pros::Motor` vector from a constexpr array of `MotorSlot` (any length). */
template <std::size_t N>
std::vector<pros::Motor> make_motors(const std::array<example_roller_constants::MotorSlot, N>& slots) {
	std::vector<pros::Motor> out;
	out.reserve(N);
	for (const auto& s : slots) {
		out.emplace_back(s.port, s.gear);
	}
	return out;
}

} // namespace example_roller

/**
 * Multi-motor velocity roller: same target RPM for every motor, velocity PID on average speed,
 * optional feedforward. Pass 1+ motors (different gears per motor via `MotorSlot` when building).
 */
class ExampleVelocityRoller : public Subsystem {
public:
	explicit ExampleVelocityRoller(std::vector<pros::Motor> motors)
		: motors_(std::move(motors)),
		  pid_(example_roller_constants::kVelocityKp, example_roller_constants::kVelocityKi,
		       example_roller_constants::kVelocityKd, example_roller_constants::kVelocityIntegralWindup) {}

	void periodic() override {
		if (motors_.empty()) {
			return;
		}
		constexpr double kDtSec = 0.01;
		double sum = 0.0;
		for (const auto& m : motors_) {
			sum += m.get_actual_velocity();
		}
		const double avg_rpm = sum / static_cast<double>(motors_.size());
		const double err = target_rpm_ - avg_rpm;
		const double ff = example_roller_constants::kVelocityFfMvPerRpm * target_rpm_;
		const double u = ff + pid_.update(err, kDtSec);
		const auto mv = static_cast<std::int32_t>(std::clamp(
			u, -static_cast<double>(example_roller_constants::kMaxVoltageMv),
			static_cast<double>(example_roller_constants::kMaxVoltageMv)));
		for (auto& m : motors_) {
			m.move_voltage(mv);
		}
	}

	/** Closed-loop: target RPM (same sign as wheel direction). */
	void setTargetRpm(double rpm) {
		target_rpm_ = std::clamp(rpm, -example_roller_constants::kMaxAbsRpm, example_roller_constants::kMaxAbsRpm);
	}

	/** [-1, 1] → ±`kMaxAbsRpm`. */
	void setTargetPct(double pct) {
		pct = std::clamp(pct, -1.0, 1.0);
		setTargetRpm(pct * example_roller_constants::kMaxAbsRpm);
	}

	void stop() {
		target_rpm_ = 0;
		pid_.reset();
		for (auto& m : motors_) {
			m.move_voltage(0);
		}
	}

	double targetRpm() const { return target_rpm_; }

	RunCommand* velocityPctCommand(double pct) {
		return new RunCommand([this, pct]() { setTargetPct(pct); }, {this});
	}

	/** Same as `velocityPctCommand` (legacy name). */
	RunCommand* pctCommand(double pct) { return velocityPctCommand(pct); }

	RunCommand* velocityRpmCommand(double rpm) {
		return new RunCommand([this, rpm]() { setTargetRpm(rpm); }, {this});
	}

private:
	std::vector<pros::Motor> motors_;
	control::PidController pid_;
	double target_rpm_ = 0;
};

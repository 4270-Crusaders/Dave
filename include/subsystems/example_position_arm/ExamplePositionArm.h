#pragma once

#include "subsystems/example_position_arm/ExamplePositionArmConstants.h"
#include "utils/command/command.h"
#include "utils/command/functionalCommand.h"
#include "utils/command/runCommand.h"
#include "utils/command/subsystem.h"
#include "utils/control/PidController.h"
#include "pros/motors.hpp"

#include <algorithm>
#include <cmath>

/** Template: single-motor arm holding angle (deg) with PID; periodic runs the loop. */
class ExamplePositionArm : public Subsystem {
public:
	explicit ExamplePositionArm(pros::Motor motor)
		: motor_(std::move(motor)),
		  pid_(example_position_arm_constants::kKp, example_position_arm_constants::kKi,
		       example_position_arm_constants::kKd, example_position_arm_constants::kIntegralWindup) {
		motor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		target_degrees_ = motor_.get_position();
	}

	void periodic() override {
		constexpr double kDtSec = 0.01;
		const double err = target_degrees_ - motor_.get_position();
		const double out = pid_.update(err, kDtSec);
		const std::int32_t mv = static_cast<std::int32_t>(std::clamp(
			out * 1000.0, -static_cast<double>(example_position_arm_constants::kMaxVoltageMv),
			static_cast<double>(example_position_arm_constants::kMaxVoltageMv)));
		motor_.move_voltage(mv);
	}

	void setTargetDegrees(double degrees) { target_degrees_ = degrees; }
	double getPositionDegrees() const { return motor_.get_position(); }
	bool atTarget() const {
		return std::abs(motor_.get_position() - target_degrees_) < example_position_arm_constants::kAngleToleranceDeg;
	}

	RunCommand* holdCommand() { return new RunCommand([]() {}, {this}); }

	Command* goToDegreesCommand(double degrees) {
		return new FunctionalCommand(
			[this, degrees]() {
				setTargetDegrees(degrees);
				pid_.reset();
			},
			[]() {},
			[this](bool) {},
			[this]() { return atTarget(); },
			{this});
	}

private:
	pros::Motor motor_;
	control::PidController pid_;
	double target_degrees_ = 0;
};

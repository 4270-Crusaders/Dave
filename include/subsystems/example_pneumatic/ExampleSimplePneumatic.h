#pragma once

#include "subsystems/example_pneumatic/ExampleSimplePneumaticConstants.h"
#include "utils/command/runCommand.h"
#include "utils/command/subsystem.h"
#include "pros/adi.hpp"

/** Template: single solenoid, two states. */
class ExampleSimplePneumatic : public Subsystem {
public:
	enum class State { Retracted, Extended };

	explicit ExampleSimplePneumatic(pros::adi::Pneumatics piston) : piston_(std::move(piston)) {}

	void periodic() override {}

	void setState(State s) {
		current_ = s;
		if (s == State::Extended) {
			piston_.extend();
		} else {
			piston_.retract();
		}
	}

	State state() const { return current_; }

	RunCommand* setCommand(State s) {
		return new RunCommand([this, s]() { setState(s); }, {this});
	}

private:
	pros::adi::Pneumatics piston_;
	State current_ = State::Retracted;
};

#pragma once

#include "command/subsystem.h"
#include "command/runCommand.h"
#include "pros/adi.hpp"

/**
 * Lever subsystem
 *
 * Allows controlling the stage of the lever subsystem
 */
class Lever : public Subsystem {
private:
	/**
	 * Store the necessary resources for this subsystem in the private section. We will control these resources in
	 * member methods
	 */
	pros::adi::Pneumatics leverPiston1;
	pros::adi::Pneumatics leverPiston2;
	
public:
	enum class LeverState {
		Store,
		Up,
		Low
	}
	currentState = LeverState::Store;
	/**
	 * Construct a new lever subsystem with pro::adi::Pneumatics object
	 *
	 * @param leverPiston1
	 * @param leverPiston2
	 */
	explicit Lever(pros::adi::Pneumatics leverPiston1, pros::adi::Pneumatics leverPiston2)
		: leverPiston1(std::move(leverPiston1)), leverPiston2(std::move(leverPiston2)) {
	}

	/**
	 * @brief Periodic runs every frame (10ms) by the command scheduler after the subsystem is registered.
	 *
	 * @note Periodic is very useful tasks such as PID that need to run every frame, but the target is only set once.
	 */
	void periodic() override {
		// EX: debugging tasks
		std::cout << "Lever Piston 1: " << leverPiston1.is_extended() << std::endl;
		std::cout << "Lever Piston 2: " << leverPiston2.is_extended() << std::endl;

		// Also:
		// Updating PID for something like a flywheel or odometry for a drivetrain subsystem
	}

	// Create member functions to actuate the subsystem
	// EX: move the motor at a certain (signed) percentage of voltage.
	/**
	 * @brief This command moves the lever to a certain state
	 *
	 * @param state LeverState for the lever in the range [Store, Up, Low]
	 */
	void setState(const LeverState state) {
		switch(state) {
			case LeverState::Store:
				leverPiston1.set_value(false);
				leverPiston2.set_value(true);
				break;
			case LeverState::Up:
				leverPiston1.set_value(true);
				leverPiston2.set_value(true);
				break;
			case LeverState::Low:
				leverPiston1.set_value(false);
				leverPiston2.set_value(false);
				break;
		}
		currentState = state;
	}

	/**
	 * @brief Command that sets the lever to a certain state
	 * This is very useful for simple commands or default commands
	 *
	 * @param state LeverState Desired for this command
	 * @return Command pointer to a RunCommand that moves the lever
	 */
	RunCommand* setCommand(const LeverState state) {
		// Create a new RunCommand
		// The lambda body is called at every update, in this case setting the lever state
		return new RunCommand(
			[this, state] () // Capture "this" and the state request
			{
				this->setState(state); // Set the state of the lever to the request
			},
			{this} // Add "this", the pointer to this subsystem that is currently running.
			       // It is important to ensure that all subsystems that are being utilized in a command are properly
			       // Freed to allow that command to run.
			);
	}

	// Free any additional resources that are needed.
	~Lever() override = default;
};
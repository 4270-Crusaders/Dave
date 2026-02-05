#pragma once

#include "command/subsystem.h"
#include "command/runCommand.h"
#include "pros/adi.hpp"

/**
 * Lever subsystem
 *
 * Allows controlling the stage of the lever subsystem
 */
class MatchLoader : public Subsystem {
private:
	/**
	 * Store the necessary resources for this subsystem in the private section. We will control these resources in
	 * member methods
	 */
	pros::adi::Pneumatics MatchLoaderPiston;

public:
	enum class MatchLoaderState {
		Up,
		Down
	}
	currentState = MatchLoaderState::Down;
	/**
	 * Construct a new lever subsystem with pro::adi::Pneumatics object
	 *
	 * @param DescorePiston
	 */
	explicit MatchLoader(pros::adi::Pneumatics MatchLoaderPiston)
		: MatchLoaderPiston(std::move(MatchLoaderPiston)) {
	}

	/**
	 * @brief Periodic runs every frame (10ms) by the command scheduler after the subsystem is registered.
	 *
	 * @note Periodic is very useful tasks such as PID that need to run every frame, but the target is only set once.
	 */
	void periodic() override {
		// EX: debugging tasks
		std::cout << "Match Loader Piston: " << MatchLoaderPiston.is_extended() << std::endl;

		// Also:
		// Updating PID for something like a flywheel or odometry for a drivetrain subsystem
	}

	// Create member functions to actuate the subsystem
	// EX: move the motor at a certain (signed) percentage of voltage.
	/**
	 * @brief This command moves the lever to a certain state
	 *
	 * @param state MatchLoaderState for the match loader mechanism in the range [Up, Down]
	 */
	void setState(const MatchLoaderState state) {
		switch(state) {
			case MatchLoaderState::Up:
				MatchLoaderPiston.set_value(true);
				break;
			case MatchLoaderState::Down:
				MatchLoaderPiston.set_value(false);
				break;
		}
		currentState = state;
	}

	/**
	 * @brief Command that sets the lever to a certain state
	 * This is very useful for simple commands or default commands
	 *
	 * @param state LeverState Desired for this command
	 * @return Command pointer to a RunCommand that moves the match loader mechanism
	 */
	RunCommand* setCommand(const MatchLoaderState state) {
		// Create a new RunCommand
		// The lambda body is called at every update, in this case setting the match loader state
		return new RunCommand(
			[this, state] () // Capture "this" and the state request
			{
				this->setState(state); // Set the state of the match loader mechanism to the request
			},
			{this} // Add "this", the pointer to this subsystem that is currently running.
			       // It is important to ensure that all subsystems that are being utilized in a command are properly
			       // Freed to allow that command to run.
			);
	}

	// Free any additional resources that are needed.
	~MatchLoader() override = default;
};
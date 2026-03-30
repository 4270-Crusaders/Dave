#pragma once

#include "api.h"
#include "command.h"
#include "units/units.hpp"

/**
 * @brief Creates a \refitem Command with no requirements that finishes after a user-specified duration
 */
class WaitCommand : public Command {
	Time startTime{};
	Time duration{};
public:
	/**
	 * @brief Creates a new WaitCommand that runs for a user-specified duration
	 *
	 * @param duration LemLib time quantity (e.g. `2_sec`, `500_msec`) for this \refitem Command
	 */
	explicit WaitCommand(const Time &duration)
		: duration(duration) {
	}

	/**
	 * @brief Initializes the WaitCommand and sets the start time of the WaitCommand
	 */
	void initialize() override {
		startTime = from_msec(Number(static_cast<double>(pros::millis())));
	}

	/**
	 * @brief Returns when the WaitCommand's duration has passed
	 *
	 * @return Returns true if the duration has passed, false otherwise
	 */
	bool isFinished() override {
		return from_msec(Number(static_cast<double>(pros::millis()))) - startTime > duration;
	}

	~WaitCommand() override = default;
};

inline Command *Command::withTimeout(const Time duration) {
	return new ParallelRaceGroup({new WaitCommand(duration), this});
}


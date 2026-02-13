#pragma once

#include "command.h"
#include <algorithm>

/**
 * @brief Command that runs multiple \refitem Commands in sequence (one after another).
 */
class SequentialCommandGroup : public Command {
private:
	size_t index = 0;
	std::vector<Command*> commands;

public:
	/**
	 * Creates a new SequentialCommandGroup that runs a series of commands one after another.
	 *
	 * @param commands Initializer list for sequence Commands
	 */
	SequentialCommandGroup(const std::initializer_list<Command*> commands) : commands(commands) {}

	/**
	 * @brief Initializes the first command
	 */
	void initialize() override {
		index = 0;
		commands[0]->initialize();
	}

	/**
	 * @brief Execute the current command, and step through when it's done
	 */
	void execute() override {
		commands[index]->execute();

		if (commands[index]->isFinished()) {
			commands[index]->end(false);
			index++;
			if (index < commands.size()) {
				commands[index]->initialize();
			}
		}
	}

	/**
	 * Finishes when the last command is finished
	 *
	 * @return true if the last command has completed
	 */
	bool isFinished() override {
		return index >= commands.size();
	}

	/**
	 * @brief Ends the current command when the SequentialCommandGroup is interrupted
	 *
	 * @param interrupted End the current command with interrupted=true if the group was interrupted
	 */
	void end(const bool interrupted) override {
		if (index < commands.size()) {
			commands[index]->end(interrupted);
		}
	}

	/**
	 * @brief Returns the union of requirements of all commands in the sequence.
	 *
	 * @return Requirements needed for each step (all subsystems used by any command in the sequence)
	 */
	std::vector<Subsystem*> getRequirements() override {
		std::vector<Subsystem*> requirements;

		for (auto* command : commands) {
			for (auto subsystem : command->getRequirements()) {
				if (std::ranges::find(requirements, subsystem) == requirements.end()) {
					requirements.emplace_back(subsystem);
				}
			}
		}

		return requirements;
	}
};

inline Command* Command::andThen(Command* other) {
	return new SequentialCommandGroup({this, other});
}

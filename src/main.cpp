#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"

CommandController primary(pros::E_CONTROLLER_MASTER);

Drive *chassis = nullptr;

ExampleVelocityRoller *exampleRoller = nullptr;
ExampleSimplePneumatic *examplePneumatic = nullptr;
ExamplePositionArm *exampleArm = nullptr;

[[noreturn]] void update_loop() {
	while (true) {
		std::uint32_t start_time = pros::millis();
		CommandScheduler::run();
		pros::c::task_delay_until(&start_time, 10);
	}
}

void initialize() {
	chassis = new Drive();

	CommandScheduler::registerSubsystem(chassis, DriveCommands::arcadeDefaultCommand(chassis, &primary));

	if constexpr (robot::kEnableExampleSubsystems) {
		exampleRoller = new ExampleVelocityRoller(
			example_roller::make_motors(example_roller_constants::kMotorSlots));
		examplePneumatic = new ExampleSimplePneumatic(pros::adi::Pneumatics(
			example_pneumatic_constants::kAdiPort, example_pneumatic_constants::kDefaultExtended));
		exampleArm = new ExamplePositionArm(pros::Motor(example_position_arm_constants::kMotorPort));
		CommandScheduler::registerSubsystem(exampleRoller, exampleRoller->velocityPctCommand(0.0));
		CommandScheduler::registerSubsystem(
			examplePneumatic, examplePneumatic->setCommand(ExampleSimplePneumatic::State::Retracted));
		CommandScheduler::registerSubsystem(exampleArm, exampleArm->holdCommand());
	}

	pros::Task commandSchedulerTask(update_loop);

	chassis->calibrate(true);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}

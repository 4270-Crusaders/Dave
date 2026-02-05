#include "main.h"
#include "command/parallelCommandGroup.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"

CommandController primary(pros::E_CONTROLLER_MASTER);

Intake *intake;
DescoreMech *descoreMech;
MatchLoader *matchLoader;
Lever *lever;
Drive *drive;

/**
 * @brief This function runs the update scheduler at each frame with a consistent schedule
 *
 * @warning This function or alternative similar to it must be running to ensure the \refitem CommandScheduler is run
 */
[[noreturn]] void update_loop() {
	// Loop forever
	while (true) {
		// Store the start time
		auto start_time = pros::millis();

		// Run the command scheduler
		// This might be an expensive(Time wise) computation
		CommandScheduler::run();

		// Use delay until if this computation ends up being expensive, keeping loop time in check
		pros::c::task_delay_until(&start_time, 10);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// Start the command scheduler task
	pros::Task commandSchedulerTask(update_loop);
	// Create subsystem instances
	// Create motor groups and IMU first
    pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
    pros::MotorGroup rightMotors({18,19,-20}, pros::MotorGearset::blue);
    pros::Imu driveImu(16);

    // Create subsystem instances
    drive = new Drive(leftMotors, rightMotors, driveImu);

	intake = new Intake(pros::Motor(14),pros::Motor(-17));
	descoreMech = new DescoreMech(pros::adi::Pneumatics('G', false));
	matchLoader = new MatchLoader(pros::adi::Pneumatics('H', false));
	lever = new Lever(pros::adi::Pneumatics('A', false), pros::adi::Pneumatics('B', false));
	
	// pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
	// pros::MotorGroup rightMotors({18,19,-20}, pros::MotorGearset::blue);

	// Register subsystems with the command scheduler
	CommandScheduler::registerSubsystem(intake, intake->pctCommand(0.0));
	CommandScheduler::registerSubsystem(descoreMech, descoreMech->setCommand(DescoreMech::DescoreState::Up));
	CommandScheduler::registerSubsystem(matchLoader, matchLoader->setCommand(MatchLoader::MatchLoaderState::Down));
	CommandScheduler::registerSubsystem(lever, lever->setCommand(Lever::LeverState::Store));
	
	// Intake Sequencewhile L1 is true
	primary.getTrigger(DIGITAL_L1)->whileTrue(new ParallelCommandGroup({
		intake->pctCommand(1.0),
		lever->setCommand(Lever::LeverState::Store),
		matchLoader->setCommand(MatchLoader::MatchLoaderState::Up),
	}));

	// Intake with MatchLoader Sequence while L2 is true
	primary.getTrigger(DIGITAL_L2)->whileTrue(new ParallelCommandGroup({
		intake->pctCommand(1.0),
		lever->setCommand(Lever::LeverState::Store),
		matchLoader->setCommand(MatchLoader::MatchLoaderState::Down),
	}));

	// HighGoal Sequence while R1 is true
	primary.getTrigger(DIGITAL_R1)->whileTrue(new ParallelCommandGroup({
		intake->pctCommand(1.0),
		lever->setCommand(Lever::LeverState::Up),
		matchLoader->setCommand(MatchLoader::MatchLoaderState::Up),
	}));

	// LowGoal Sequence while R2 is true
	primary.getTrigger(DIGITAL_R2)->whileTrue(new ParallelCommandGroup({
		intake->pctCommand(1.0),
		lever->setCommand(Lever::LeverState::Low),
		matchLoader->setCommand(MatchLoader::MatchLoaderState::Up),
	}));

	// Outtake while A is true
	primary.getTrigger(DIGITAL_A)->whileTrue(new ParallelCommandGroup({
		intake->pctCommand(-1.0),
		lever->setCommand(Lever::LeverState::Up),
		matchLoader->setCommand(MatchLoader::MatchLoaderState::Up),
	}));


	// // Toggle pctCommand to run while R1 turns to ture
	// primary.getTrigger(DIGITAL_R2)->toggleOnTrue(intake->pctCommand(1.0));

	// // Dejam mode, causes the intake to move back and forth quickly
	// primary.getTrigger(DIGITAL_A)->whileTrue(
	// 													intake->pctCommand(-1.0)
	// 													->withTimeout(300_ms)
	// 													->andThen(intake->pctCommand(1.0)
	// 													->withTimeout(300_ms))
	// 													->repeatedly());
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		drive->getChassis()->arcade(primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		pros::delay(10);
	}
}
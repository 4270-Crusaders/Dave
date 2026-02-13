#include "main.h"
#include "command/commandController.h"
#include "command/functionalCommand.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "subsystems/drive.h"

Drive::Drive()
	: leftMotors({-11, 12, -13}, pros::MotorGearset::blue),
	  rightMotors({18, 19, -20}, pros::MotorGearset::blue),
	  imu(16),
	  horizontalEnc(6),
	  verticalEnc(-5),
	  horizontal(&horizontalEnc, 2, -3.25),
	  vertical(&verticalEnc, 2, -0.375),
	  drivetrain(&leftMotors, &rightMotors, 10, lemlib::Omniwheel::NEW_275, 450, 2),
	  linearController(10, 0, 3, 3, 1, 100, 3, 500, 20),
	  angularController(2, 0, 10, 3, 1, 100, 3, 500, 0),
	  sensors(&vertical, nullptr, &horizontal, nullptr, &imu),
	  throttleCurve(3, 10, 1.019),
	  steerCurve(3, 10, 1.019),
	  chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve) {
}

void Drive::periodic() {}

void Drive::arcade(double throttle, double turn) {
	chassis.arcade(static_cast<int>(throttle), static_cast<int>(turn));
}

void Drive::tank(int left, int right, bool disableDriveCurve) {
	chassis.tank(left, right, disableDriveCurve);
}

void Drive::curvature(int throttle, int turn, bool disableDriveCurve) {
	chassis.curvature(throttle, turn, disableDriveCurve);
}

void Drive::calibrate(bool calibrateIMU) {
	chassis.calibrate(calibrateIMU);
}

lemlib::Pose Drive::getPose(bool radians, bool standardPos) {
	return chassis.getPose(radians, standardPos);
}

void Drive::setPose(float x, float y, float theta, bool radians) {
	chassis.setPose(x, y, theta, radians);
}

void Drive::setPose(lemlib::Pose pose, bool radians) {
	chassis.setPose(pose, radians);
}

void Drive::resetLocalPosition() {
	chassis.resetLocalPosition();
}

void Drive::moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params, bool async) {
	chassis.moveToPoint(x, y, timeout, params, async);
}

void Drive::moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
	chassis.moveToPose(x, y, theta, timeout, params, async);
}

void Drive::turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async) {
	chassis.turnToHeading(theta, timeout, params, async);
}

void Drive::turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async) {
	chassis.turnToPoint(x, y, timeout, params, async);
}

void Drive::swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                           lemlib::SwingToHeadingParams params, bool async) {
	chassis.swingToHeading(theta, lockedSide, timeout, params, async);
}

void Drive::swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                         lemlib::SwingToPointParams params, bool async) {
	chassis.swingToPoint(x, y, lockedSide, timeout, params, async);
}

void Drive::follow(const asset& path, float lookahead, int timeout, bool forwards, bool async) {
	chassis.follow(path, lookahead, timeout, forwards, async);
}

void Drive::waitUntilDone() {
	chassis.waitUntilDone();
}

void Drive::waitUntil(float dist) {
	chassis.waitUntil(dist);
}

bool Drive::isInMotion() const {
	return chassis.isInMotion();
}

void Drive::cancelMotion() {
	chassis.cancelMotion();
}

void Drive::cancelAllMotions() {
	chassis.cancelAllMotions();
}

void Drive::setBrakeMode(pros::motor_brake_mode_e mode) {
	chassis.setBrakeMode(mode);
}

RunCommand* Drive::arcadeCommand(CommandController* controller) {
	return new RunCommand(
		[this, controller]() {
			arcade(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
			       controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		},
		{this});
}


RunCommand* Drive::tankCommand(CommandController* controller, bool disableDriveCurve){
	return new RunCommand(
		[this, controller,disableDriveCurve](){
			tank(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
					controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
					disableDriveCurve);
	},
	{this});
}

RunCommand* Drive::curvatureCommand(CommandController* controller, bool disableDriveCurve){
	return new RunCommand(
		[this, controller, disableDriveCurve](){
			curvature(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
					controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
					disableDriveCurve);
	},
	{this});
}

// Command factories: start motion in init, finish when !isInMotion()
Command* Drive::moveToPointCommand(float x, float y, int timeout, lemlib::MoveToPointParams params) {
	return new FunctionalCommand(
		[this, x, y, timeout, params]() { moveToPoint(x, y, timeout, params, true); },
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::moveToPoseCommand(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params) {
	return new FunctionalCommand(
		[this, x, y, theta, timeout, params]() { moveToPose(x, y, theta, timeout, params, true); },
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::turnToHeadingCommand(float theta, int timeout, lemlib::TurnToHeadingParams params) {
	return new FunctionalCommand(
		[this, theta, timeout, params]() { turnToHeading(theta, timeout, params, true); },
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::turnToPointCommand(float x, float y, int timeout, lemlib::TurnToPointParams params) {
	return new FunctionalCommand(
		[this, x, y, timeout, params]() { turnToPoint(x, y, timeout, params, true); },
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::swingToHeadingCommand(float theta, lemlib::DriveSide lockedSide, int timeout,
                                      lemlib::SwingToHeadingParams params) {
	return new FunctionalCommand(
		[this, theta, lockedSide, timeout, params]() {
			swingToHeading(theta, lockedSide, timeout, params, true);
		},
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::swingToPointCommand(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                                    lemlib::SwingToPointParams params) {
	return new FunctionalCommand(
		[this, x, y, lockedSide, timeout, params]() {
			swingToPoint(x, y, lockedSide, timeout, params, true);
		},
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

Command* Drive::followPathCommand(const asset& path, float lookahead, int timeout, bool forwards) {
	return new FunctionalCommand(
		[this, path, lookahead, timeout, forwards]() { follow(path, lookahead, timeout, forwards, true); },
		[]() {},
		[this](bool interrupted) { if (interrupted) cancelMotion(); },
		[this]() { return !isInMotion(); },
		{this});
}

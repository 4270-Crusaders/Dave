#pragma once

#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "utils/command/subsystem.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

/**
 * Drive subsystem: thin wrapper around LemLib `Chassis` (odometry, teleop, autonomous motions).
 * Command factories: `commands/DriveCommands.h`.
 */
class Drive : public Subsystem {
public:
	Drive();

	void periodic() override;

	void arcade(double throttle, double turn);
	void tank(int left, int right);
	void curvature(int throttle, int turn);

	void calibrate(bool calibrateImu = true);

	lemlib::Pose getPose(bool radians = false, bool standardPos = false);
	void setPose(float x, float y, float theta, bool radians = false);
	void setPose(lemlib::Pose pose, bool radians = false);
	void resetLocalPosition();

	void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true);
	void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = true);
	/** `moveToPose` with a stronger carrot (higher `lead`) for wider arcs. */
	void moveToPoseBoomerang(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {},
	                         bool async = true);

	void turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
	void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true);
	void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
	                    lemlib::SwingToHeadingParams params = {}, bool async = true);
	void swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
	                  lemlib::SwingToPointParams params = {}, bool async = true);

	/** Pure pursuit: path text file in `static/`, bound with the LemLib ASSET macro. */
	void follow(const asset& path, float lookahead, int timeout, bool forwards = true, bool async = true);

	void waitUntilDone();
	void waitUntil(float dist);
	bool isInMotion() const;
	void cancelMotion();
	void cancelAllMotions();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);

	lemlib::Chassis& chassis() { return chassis_; }
	const lemlib::Chassis& chassis() const { return chassis_; }

private:
	pros::MotorGroup left_motors_;
	pros::MotorGroup right_motors_;
	pros::Imu imu_;
	pros::Rotation vertical_enc_;
	pros::Rotation horizontal_enc_;
	lemlib::TrackingWheel vertical_wheel_;
	lemlib::TrackingWheel horizontal_wheel_;
	lemlib::Drivetrain drivetrain_;
	lemlib::ControllerSettings linear_settings_;
	lemlib::ControllerSettings angular_settings_;
	lemlib::OdomSensors sensors_;
	lemlib::ExpoDriveCurve throttle_curve_;
	lemlib::ExpoDriveCurve steer_curve_;
	lemlib::Chassis chassis_;
};

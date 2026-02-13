#pragma once

#include "command/command.h"
#include "command/runCommand.h"
#include "command/subsystem.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class CommandController;

/**
 * Drive subsystem
 *
 * Wraps the lemlib chassis: odometry, driver control (arcade/tank/curvature),
 * and autonomous motions (moveToPoint, turnToHeading, follow path). Use
 * command factories for use in SequentialCommandGroup / autonomous.
 */
class Drive : public Subsystem {
public:
	Drive();

	void periodic() override;

	// ----- Driver control -----
	/** Arcade: throttle (e.g. left Y) and turn (e.g. right X), [-127, 127]. */
	void arcade(double throttle, double turn);
	/** Tank: left and right stick, [-127, 127]. */
	void tank(int left, int right, bool disableDriveCurve = false);
	/** Curvature: throttle and curvature (radius), [-127, 127]. */
	void curvature(int throttle, int turn, bool disableDriveCurve = false);

	/** Default command for teleop: arcade from controller. */
	RunCommand* arcadeCommand(CommandController* controller);
	RunCommand* tankCommand(CommandController* controller, bool disableDriveCurve);
	RunCommand* curvatureCommand(CommandController* controller, bool disableDriveCurve);

	// ----- Calibration & pose -----
	void calibrate(bool calibrateIMU = true);
	/** Get current pose (theta in degrees by default). */
	lemlib::Pose getPose(bool radians = false, bool standardPos = false);
	/** Set pose (theta in degrees by default). */
	void setPose(float x, float y, float theta, bool radians = false);
	void setPose(lemlib::Pose pose, bool radians = false);
	/** Reset x,y to 0 without changing heading. */
	void resetLocalPosition();

	// ----- Autonomous motions (async by default; use commands to wait) -----
	void moveToPoint(float x, float y, int timeout,
	                 lemlib::MoveToPointParams params = {}, bool async = true);
	void moveToPose(float x, float y, float theta, int timeout,
	                lemlib::MoveToPoseParams params = {}, bool async = true);
	void turnToHeading(float theta, int timeout,
	                   lemlib::TurnToHeadingParams params = {}, bool async = true);
	void turnToPoint(float x, float y, int timeout,
	                 lemlib::TurnToPointParams params = {}, bool async = true);
	void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
	                    lemlib::SwingToHeadingParams params = {}, bool async = true);
	void swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
	                  lemlib::SwingToPointParams params = {}, bool async = true);
	void follow(const asset& path, float lookahead, int timeout,
	            bool forwards = true, bool async = true);

	// ----- Motion state & control -----
	void waitUntilDone();
	void waitUntil(float dist);
	bool isInMotion() const;
	void cancelMotion();
	void cancelAllMotions();

	void setBrakeMode(pros::motor_brake_mode_e mode);

	// ----- Command factories (return commands that require this subsystem and finish when motion done) -----
	Command* moveToPointCommand(float x, float y, int timeout,
	                            lemlib::MoveToPointParams params = {});
	Command* moveToPoseCommand(float x, float y, float theta, int timeout,
	                          lemlib::MoveToPoseParams params = {});
	Command* turnToHeadingCommand(float theta, int timeout,
	                              lemlib::TurnToHeadingParams params = {});
	Command* turnToPointCommand(float x, float y, int timeout,
	                            lemlib::TurnToPointParams params = {});
	Command* swingToHeadingCommand(float theta, lemlib::DriveSide lockedSide, int timeout,
	                                lemlib::SwingToHeadingParams params = {});
	Command* swingToPointCommand(float x, float y, lemlib::DriveSide lockedSide, int timeout,
	                             lemlib::SwingToPointParams params = {});
	Command* followPathCommand(const asset& path, float lookahead, int timeout,
	                           bool forwards = true);

	~Drive() override = default;

private:
	pros::MotorGroup leftMotors;
	pros::MotorGroup rightMotors;
	pros::Imu imu;
	pros::Rotation horizontalEnc;
	pros::Rotation verticalEnc;
	lemlib::TrackingWheel horizontal;
	lemlib::TrackingWheel vertical;
	lemlib::Drivetrain drivetrain;
	lemlib::ControllerSettings linearController;
	lemlib::ControllerSettings angularController;
	lemlib::OdomSensors sensors;
	lemlib::ExpoDriveCurve throttleCurve;
	lemlib::ExpoDriveCurve steerCurve;
	lemlib::Chassis chassis;
};

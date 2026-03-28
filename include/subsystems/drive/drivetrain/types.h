#pragma once

#include "utils/geom/pose2d.h"

#include <cstdint>
#include <vector>

namespace drivetrain {

/** Legacy flat pose (inches, radians) — alias of geom::FlatPose2d for odometry / MCL. */
using Pose = geom::FlatPose2d;

struct MoveToPointParams {
	float maxSpeed = 127.f;
	float minSpeed = 20.f;
};

struct MoveToPoseParams {
	float maxSpeed = 127.f;
	float minSpeed = 20.f;
	float turnMaxSpeed = 127.f;
};

struct MoveToPoseBoomerangParams {
	bool reversed = false;
	float lead = 0.6f;
	/** Distance at which we switch from carrot-heading to final-heading (inches). */
	float closeRangeIn = 7.5f;
	/** Consider finished when dist < this AND heading error < `settleAngleDeg` (inches). */
	float settleDistIn = 1.75f;
	/** Consider finished when dist < `settleDistIn` AND heading error < this (degrees). */
	float settleAngleDeg = 2.5f;
	float maxLateralSpeed = 127.f;
	float minLateralSpeed = 0.f;
	float maxAngularSpeed = 127.f;
	float lateralSlew = 0.f;
	float angularSlew = 0.f;
	float driftCompensation = 0.f;
	/** Extra distance past the target plane where we can early-exit (inches). */
	float earlyExitRangeIn = 0.f;
};

struct TurnToHeadingParams {
	float maxSpeed = 127.f;
	float minSpeed = 15.f;
};

struct TurnToPointParams {
	float maxSpeed = 127.f;
	float minSpeed = 15.f;
};

enum class DriveSide { Left, Right };

struct SwingToHeadingParams {
	float maxSpeed = 127.f;
	float minSpeed = 15.f;
};

struct SwingToPointParams {
	float maxSpeed = 127.f;
	float minSpeed = 15.f;
};

struct Waypoint {
	double x = 0;
	double y = 0;
};

using Path = std::vector<Waypoint>;

} // namespace drivetrain

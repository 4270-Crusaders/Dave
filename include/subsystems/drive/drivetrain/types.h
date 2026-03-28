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

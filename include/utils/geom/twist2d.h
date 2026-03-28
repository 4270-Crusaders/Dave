#pragma once

namespace geom {

/** 2D velocity / displacement twist (dx, dy, dtheta). Units consistent with pose (e.g. in per s, rad/s). */
struct Twist2d {
	double dx = 0;
	double dy = 0;
	double dtheta = 0;
};

} // namespace geom

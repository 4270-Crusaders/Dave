#pragma once

/** Shared math for geometry (radians; translations in inches for VEX unless noted). */

#include <algorithm>
#include <cmath>

namespace geom {

inline constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double deg) { return deg * kPi / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / kPi; }

/** Wrap to (-pi, pi]. */
inline double angleWrap(double a) {
	while (a <= -kPi) {
		a += 2.0 * kPi;
	}
	while (a > kPi) {
		a -= 2.0 * kPi;
	}
	return a;
}

inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

} // namespace geom

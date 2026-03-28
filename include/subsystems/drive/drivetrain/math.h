#pragma once

#include <algorithm>
#include <cmath>

namespace drivetrain {

inline constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double deg) { return deg * kPi / 180.0; }

inline double radToDeg(double rad) { return rad * 180.0 / kPi; }

/** Wrap angle to (-pi, pi]. */
inline double normalizeAngleRad(double a) {
	while (a <= -kPi) {
		a += 2.0 * kPi;
	}
	while (a > kPi) {
		a -= 2.0 * kPi;
	}
	return a;
}

inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

} // namespace drivetrain

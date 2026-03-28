#pragma once

#include <cmath>

namespace localization {

/** Distance along ray from (x,y) at angle `ray_angle` (rad) to first hit on axis-aligned box [min_x,max_x]×[min_y,max_y]. */
inline double raycastAxisAlignedRectangle(double x, double y, double ray_angle, double min_x, double min_y,
                                            double max_x, double max_y) {
	const double dx = std::cos(ray_angle);
	const double dy = std::sin(ray_angle);
	double best = 1e9;

	auto try_t = [&](double t) {
		if (t > 1e-4 && t < best) {
			best = t;
		}
	};

	if (std::abs(dx) > 1e-9) {
		try_t((min_x - x) / dx);
		try_t((max_x - x) / dx);
	}
	if (std::abs(dy) > 1e-9) {
		try_t((min_y - y) / dy);
		try_t((max_y - y) / dy);
	}

	if (best > 1e8) {
		return 1e6;
	}
	const double hx = x + best * dx;
	const double hy = y + best * dy;
	const bool inside = hx >= min_x - 1e-6 && hx <= max_x + 1e-6 && hy >= min_y - 1e-6 && hy <= max_y + 1e-6;
	return inside ? best : 1e6;
}

} // namespace localization

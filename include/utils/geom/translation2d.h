#pragma once

#include "utils/geom/math.h"
#include "utils/geom/rotation2d.h"

namespace geom {

class Translation2d {
public:
	constexpr Translation2d() = default;
	constexpr Translation2d(double x, double y) : m_x(x),
	                                               m_y(y) {}

	double x() const { return m_x; }
	double y() const { return m_y; }
	double norm() const { return std::hypot(m_x, m_y); }

	Translation2d operator+(const Translation2d& o) const { return {m_x + o.m_x, m_y + o.m_y}; }
	Translation2d operator-(const Translation2d& o) const { return {m_x - o.m_x, m_y - o.m_y}; }
	Translation2d operator-() const { return {-m_x, -m_y}; }
	Translation2d operator*(double s) const { return {m_x * s, m_y * s}; }

	double dot(const Translation2d& o) const { return m_x * o.m_x + m_y * o.m_y; }

	/** Rotate this vector by rotation (field / robot frame). */
	Translation2d rotateBy(const Rotation2d& rot) const {
		const double c = rot.cos();
		const double s = rot.sin();
		return {m_x * c - m_y * s, m_x * s + m_y * c};
	}

	bool operator==(const Translation2d& o) const { return m_x == o.m_x && m_y == o.m_y; }

private:
	double m_x = 0;
	double m_y = 0;
};

} // namespace geom

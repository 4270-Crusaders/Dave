#pragma once

#include "utils/geom/math.h"

namespace geom {

class Translation3d {
public:
	constexpr Translation3d() = default;
	constexpr Translation3d(double x, double y, double z) : m_x(x),
	                                                         m_y(y),
	                                                         m_z(z) {}

	double x() const { return m_x; }
	double y() const { return m_y; }
	double z() const { return m_z; }
	double norm() const { return std::sqrt(m_x * m_x + m_y * m_y + m_z * m_z); }

	Translation3d operator+(const Translation3d& o) const { return {m_x + o.m_x, m_y + o.m_y, m_z + o.m_z}; }
	Translation3d operator-(const Translation3d& o) const { return {m_x - o.m_x, m_y - o.m_y, m_z - o.m_z}; }
	Translation3d operator-() const { return {-m_x, -m_y, -m_z}; }
	Translation3d operator*(double s) const { return {m_x * s, m_y * s, m_z * s}; }

	double dot(const Translation3d& o) const { return m_x * o.m_x + m_y * o.m_y + m_z * o.m_z; }
	Translation3d cross(const Translation3d& o) const {
		return {m_y * o.m_z - m_z * o.m_y, m_z * o.m_x - m_x * o.m_z, m_x * o.m_y - m_y * o.m_x};
	}

	bool operator==(const Translation3d& o) const { return m_x == o.m_x && m_y == o.m_y && m_z == o.m_z; }

private:
	double m_x = 0;
	double m_y = 0;
	double m_z = 0;
};

} // namespace geom

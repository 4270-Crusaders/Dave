#pragma once

#include "utils/geom/math.h"
#include "utils/geom/translation3d.h"

#include <cmath>

namespace geom {

/** Unit quaternion rotation (Hamilton convention, active rotation on vectors). */
class Rotation3d {
public:
	Rotation3d() : m_w(1),
	               m_x(0),
	               m_y(0),
	               m_z(0) {}

	Rotation3d(double w, double x, double y, double z) : m_w(w),
	                                                       m_x(x),
	                                                       m_y(y),
	                                                       m_z(z) { normalize(); }

	static Rotation3d fromRPY(double roll, double pitch, double yaw) {
		const double cr = std::cos(roll * 0.5);
		const double sr = std::sin(roll * 0.5);
		const double cp = std::cos(pitch * 0.5);
		const double sp = std::sin(pitch * 0.5);
		const double cy = std::cos(yaw * 0.5);
		const double sy = std::sin(yaw * 0.5);
		return Rotation3d(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
		                  cr * cp * sy - sr * sp * cy);
	}

	Translation3d rotate(const Translation3d& v) const {
		// v' = q * (0,v) * q_conj
		const double tx = 2.0 * (m_y * v.z() - m_z * v.y());
		const double ty = 2.0 * (m_z * v.x() - m_x * v.z());
		const double tz = 2.0 * (m_x * v.y() - m_y * v.x());
		return {v.x() + m_w * tx + m_y * tz - m_z * ty, v.y() + m_w * ty + m_z * tx - m_x * tz,
		        v.z() + m_w * tz + m_x * ty - m_y * tx};
	}

	Rotation3d operator*(const Rotation3d& q) const {
		return {m_w * q.m_w - m_x * q.m_x - m_y * q.m_y - m_z * q.m_z,
		        m_w * q.m_x + m_x * q.m_w + m_y * q.m_z - m_z * q.m_y,
		        m_w * q.m_y - m_x * q.m_z + m_y * q.m_w + m_z * q.m_x,
		        m_w * q.m_z + m_x * q.m_y - m_y * q.m_x + m_z * q.m_w};
	}

	Rotation3d inverse() const { return {m_w, -m_x, -m_y, -m_z}; }

	double w() const { return m_w; }
	double x() const { return m_x; }
	double y() const { return m_y; }
	double z() const { return m_z; }

private:
	void normalize() {
		const double n = std::sqrt(m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z);
		if (n > 1e-12) {
			m_w /= n;
			m_x /= n;
			m_y /= n;
			m_z /= n;
		}
	}

	double m_w, m_x, m_y, m_z;
};

} // namespace geom

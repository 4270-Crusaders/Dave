#pragma once

#include "utils/geom/rotation3d.h"
#include "utils/geom/translation3d.h"

namespace geom {

class Transform3d {
public:
	Transform3d() = default;
	Transform3d(Translation3d t, Rotation3d r) : m_t(std::move(t)),
	                                             m_r(std::move(r)) {}

	const Translation3d& translation() const { return m_t; }
	const Rotation3d& rotation() const { return m_r; }

	Translation3d operator*(const Translation3d& p) const { return m_r.rotate(p) + m_t; }

	Transform3d operator*(const Transform3d& o) const {
		return {m_t + m_r.rotate(o.m_t), m_r * o.m_r};
	}

	Transform3d inverse() const {
		const Rotation3d ri = m_r.inverse();
		return {ri.rotate(-m_t), ri};
	}

private:
	Translation3d m_t{};
	Rotation3d m_r{};
};

} // namespace geom

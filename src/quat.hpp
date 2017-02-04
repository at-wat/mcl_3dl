#ifndef __QUAT_HPP__
#define __QUAT_HPP__

#include <vec3.hpp>


class quat
{
public:
	float x;
	float y;
	float z;
	float w;

	quat(const float &x, const float &y, const float &z, const float &w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	quat(const vec3 &axis, const float &ang)
	{
		set_axis_ang(axis, ang);
	}
	quat(const vec3 &rpy)
	{
		set_rpy(rpy);
	}
	quat()
	{
		w = 1.0;
		x = y = z = 0.0;
	}
	float dot(const quat &q) const
	{
		return x*q.x + y*q.y + z*q.z + w*q.w;
	}
	float norm() const
	{
		return sqrtf(dot(*this));
	}
	quat operator+(const quat &q) const
	{
		return quat(x+q.x, y+q.y, z+q.z, w+q.w);
	}
	quat operator-(const quat &q) const
	{
		return quat(x-q.x, y-q.y, z-q.z, w-q.w);
	}
	quat operator*(const quat &q) const
	{
		return quat(
				w*q.x + x*q.w + y*q.z - z*q.y,
				w*q.y + y*q.w + z*q.x - x*q.z,
				w*q.z + z*q.w + x*q.y - y*q.x,
				w*q.w - x*q.x - y*q.y - z*q.z);

	}
	quat operator/(const float &s) const
	{
		return operator*(1.0 / s);
	}
	vec3 operator*(const vec3 &v) const
	{
		quat ret = *this * quat(v.x, v.y, v.z, 0.0) * conj();
		return vec3(ret.x, ret.y, ret.z);
	}
	quat operator*(const float &s) const
	{
		vec3 axis;
		float ang;
		get_axis_ang(axis, ang);
		return quat(axis, ang * s);
	}
	quat normalized() const
	{
		float n = norm();
		quat ret = *this;
		ret.x /= n;
		ret.y /= n;
		ret.z /= n;
		ret.w /= n;
		return ret;
	}
	void normalize()
	{
		*this = normalized();
	}
	quat conj() const
	{
		return quat(-x, -y, -z, w);
	}
	quat inv() const
	{
		return conj() / dot(*this);
	}
	vec3 get_rpy() const
	{
		const float ysq = y * y;
		const float t0 = -2.0 * (ysq + z * z) + 1.0;
		const float t1 = +2.0 * (x * y + w * z);
		float t2 = -2.0 * (x * z - w * y);
		const float t3 = +2.0 * (y * z + w * x);
		const float t4 = -2.0 * (x * x + ysq) + 1.0;

		if(t2 > 1.0) t2 = 1.0;
		if(t2 < -1.0) t2 = -1.0;

		return vec3(std::atan2(t3, t4), std::asin(t2), std::atan2(t1, t0));
	}
	void set_rpy(const vec3 &rpy)
	{
		const float t2 = cos(rpy.x * 0.5);
		const float t3 = sin(rpy.x * 0.5);
		const float t4 = cos(rpy.y * 0.5);
		const float t5 = sin(rpy.y * 0.5);
		const float t0 = cos(rpy.z * 0.5);
		const float t1 = sin(rpy.z * 0.5);

		x = t0 * t3 * t4 - t1 * t2 * t5;
		y = t0 * t2 * t5 + t1 * t3 * t4;
		z = t1 * t2 * t4 - t0 * t3 * t5;
		w = t0 * t2 * t4 + t1 * t3 * t5;
	}
	void set_axis_ang(const vec3 &axis, const float &ang)
	{
		const vec3 a = axis / axis.norm();
		const float s = sinf(ang / 2.0);
		this->x = a.x * s;
		this->y = a.y * s;
		this->z = a.z * s;
		this->w = cosf(ang / 2.0);
		normalize();
	}
	void get_axis_ang(vec3 &axis, float &ang) const
	{
		if(fabs(w) >= 1.0 - 0.000001)
		{
			ang = 0.0;
			axis = vec3(0.0, 0.0, 1.0);
			return;
		}
		ang = acosf(w) * 2.0;
		if(ang > M_PI) ang -= 2.0 * M_PI;
		float wsq = 1.0 - w * w;
		axis = vec3(x, y, z) / sqrtf(wsq);
	}
	void rotate_axis(const quat &r)
	{
		vec3 axis;
		float ang;
		get_axis_ang(axis, ang);
		set_axis_ang(r * axis, ang);
	}
};

#endif


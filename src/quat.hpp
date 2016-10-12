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

	quat(const float x, const float y, const float z, const float w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	quat(const vec3 axis, const float ang)
	{
		vec3 a = axis / axis.norm();
		float s = sinf(ang / 2.0);
		this->x = a.x * s;
		this->y = a.y * s;
		this->z = a.z * s;
		this->w = cosf(ang / 2.0);
	}
	quat()
	{
		x = 1.0;
		y = z = w = 0.0;
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
	quat operator*(const float &s) const
	{
		return quat(x*s, y*s, z*s, w*s);
	}
	quat operator/(const float &s) const
	{
		return quat(x/s, y/s, z/s, w/s);
	}
	vec3 operator*(const vec3 &v) const
	{
		quat ret = *this * quat(v.x, v.y, v.z, 0.0) * conj();
		return vec3(ret.x, ret.y, ret.z);
	}
	quat normalized() const
	{
		return *this / norm();
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
		float ysq = y * y;
		float t0 = -2.0 * (ysq + z * z) + 1.0;
		float t1 = +2.0 * (x * y + w * z);
		float t2 = -2.0 * (x * z - w * y);
		float t3 = +2.0 * (y * z + w * x);
		float t4 = -2.0 * (x * x + ysq) + 1.0;

		if(t2 > 1.0) t2 = 1.0;
		if(t2 < -1.0) t2 = -1.0;

		return vec3(std::atan2(t3, t4), std::asin(t2), std::atan2(t1, t0));
	}
	void set_rpy(const vec3 rpy)
	{
		float t2 = cos(rpy.x * 0.5);
		float t3 = sin(rpy.x * 0.5);
		float t4 = cos(rpy.y * 0.5);
		float t5 = sin(rpy.y * 0.5);
		float t0 = cos(rpy.z * 0.5);
		float t1 = sin(rpy.z * 0.5);

		x = t0 * t3 * t4 - t1 * t2 * t5;
		y = t0 * t2 * t5 + t1 * t3 * t4;
		z = t1 * t2 * t4 - t0 * t3 * t5;
		w = t0 * t2 * t4 + t1 * t3 * t5;
	}
};

#endif


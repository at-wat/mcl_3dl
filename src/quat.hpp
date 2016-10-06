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
};

#endif


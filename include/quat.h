/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QUAT_H
#define QUAT_H

#include <vec3.h>

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
  quat(const vec3 &forward, const vec3 &up_raw)
  {
    const vec3 xv = forward.normalized();
    const vec3 yv = up_raw.cross(xv).normalized();
    const vec3 zv = xv.cross(yv).normalized();

    w = sqrtf(std::max(0.0, 1.0 + xv.x + yv.y + zv.z)) / 2.0;
    x = sqrtf(std::max(0.0, 1.0 + xv.x - yv.y - zv.z)) / 2.0;
    y = sqrtf(std::max(0.0, 1.0 - xv.x + yv.y - zv.z)) / 2.0;
    z = sqrtf(std::max(0.0, 1.0 - xv.x - yv.y + zv.z)) / 2.0;
    if (zv.y - yv.z > 0)
      x = -x;
    if (xv.z - zv.x > 0)
      y = -y;
    if (yv.x - xv.y > 0)
      z = -z;
  }
  explicit quat(const vec3 &rpy)
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
    return x * q.x + y * q.y + z * q.z + w * q.w;
  }
  float norm() const
  {
    return sqrtf(dot(*this));
  }
  quat operator+(const quat &q) const
  {
    return quat(x + q.x, y + q.y, z + q.z, w + q.w);
  }
  quat operator-(const quat &q) const
  {
    return quat(x - q.x, y - q.y, z - q.z, w - q.w);
  }
  quat operator*(const quat &q) const
  {
    return quat(
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y + y * q.w + z * q.x - x * q.z,
        w * q.z + z * q.w + x * q.y - y * q.x,
        w * q.w - x * q.x - y * q.y - z * q.z);
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

    if (t2 > 1.0)
      t2 = 1.0;
    if (t2 < -1.0)
      t2 = -1.0;

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
    if (fabs(w) >= 1.0 - 0.000001)
    {
      ang = 0.0;
      axis = vec3(0.0, 0.0, 1.0);
      return;
    }
    ang = acosf(w) * 2.0;
    if (ang > M_PI)
      ang -= 2.0 * M_PI;
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

#endif  // QUAT_H

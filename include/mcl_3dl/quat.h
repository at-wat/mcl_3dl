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

#ifndef MCL_3DL_QUAT_H
#define MCL_3DL_QUAT_H

#include <algorithm>

#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class Quat
{
public:
  float x_;
  float y_;
  float z_;
  float w_;

  Quat(const float& x, const float& y, const float& z, const float& w)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
  }
  Quat(const Vec3& axis, const float& ang)
  {
    setAxisAng(axis, ang);
  }
  Quat(const Vec3& forward, const Vec3& up_raw)
  {
    const Vec3 xv = forward.normalized();
    const Vec3 yv = up_raw.cross(xv).normalized();
    const Vec3 zv = xv.cross(yv).normalized();

    w_ = sqrtf(std::max(0.0, 1.0 + xv.x_ + yv.y_ + zv.z_)) / 2.0;
    x_ = sqrtf(std::max(0.0, 1.0 + xv.x_ - yv.y_ - zv.z_)) / 2.0;
    y_ = sqrtf(std::max(0.0, 1.0 - xv.x_ + yv.y_ - zv.z_)) / 2.0;
    z_ = sqrtf(std::max(0.0, 1.0 - xv.x_ - yv.y_ + zv.z_)) / 2.0;
    if (zv.y_ - yv.z_ > 0)
      x_ = -x_;
    if (xv.z_ - zv.x_ > 0)
      y_ = -y_;
    if (yv.x_ - xv.y_ > 0)
      z_ = -z_;
  }
  explicit Quat(const Vec3& rpy)
  {
    setRPY(rpy);
  }
  Quat()
  {
    w_ = 1.0;
    x_ = y_ = z_ = 0.0;
  }
  float dot(const Quat& q) const
  {
    return x_ * q.x_ + y_ * q.y_ + z_ * q.z_ + w_ * q.w_;
  }
  float norm() const
  {
    return sqrtf(dot(*this));
  }
  bool operator==(const Quat& q) const
  {
    return x_ == q.x_ && y_ == q.y_ && z_ == q.z_ && w_ == q.w_;
  }
  bool operator!=(const Quat& q) const
  {
    return !operator==(q);
  }
  Quat operator+(const Quat& q) const
  {
    return Quat(x_ + q.x_, y_ + q.y_, z_ + q.z_, w_ + q.w_);
  }
  Quat operator+=(const Quat& q)
  {
    x_ += q.x_;
    y_ += q.y_;
    z_ += q.z_;
    w_ += q.w_;
    return *this;
  }
  Quat operator-(const Quat& q) const
  {
    return Quat(x_ - q.x_, y_ - q.y_, z_ - q.z_, w_ - q.w_);
  }
  Quat operator-=(const Quat& q)
  {
    x_ -= q.x_;
    y_ -= q.y_;
    z_ -= q.z_;
    w_ -= q.w_;
    return *this;
  }
  Quat operator-() const
  {
    return Quat(-x_, -y_, -z_, -w_);
  }
  Quat operator*(const Quat& q) const
  {
    return Quat(
        w_ * q.x_ + x_ * q.w_ + y_ * q.z_ - z_ * q.y_,
        w_ * q.y_ + y_ * q.w_ + z_ * q.x_ - x_ * q.z_,
        w_ * q.z_ + z_ * q.w_ + x_ * q.y_ - y_ * q.x_,
        w_ * q.w_ - x_ * q.x_ - y_ * q.y_ - z_ * q.z_);
  }
  Vec3 operator*(const Vec3& v) const
  {
    const Quat ret = *this * Quat(v.x_, v.y_, v.z_, 0.0) * conj();
    return Vec3(ret.x_, ret.y_, ret.z_);
  }
  Quat operator*(const float& s) const
  {
    return Quat(x_ * s, y_ * s, z_ * s, w_ * s);
  }
  Quat operator/(const float& s) const
  {
    return operator*(1.0 / s);
  }
  Quat operator*=(const float& s)
  {
    x_ *= s;
    y_ *= s;
    z_ *= s;
    w_ *= s;
    return *this;
  }
  Quat operator/=(const float& s)
  {
    x_ /= s;
    y_ /= s;
    z_ /= s;
    w_ /= s;
    return *this;
  }
  Quat weighted(const float& s) const
  {
    Vec3 axis;
    float ang;
    getAxisAng(axis, ang);
    return Quat(axis, ang * s);
  }
  Quat normalized() const
  {
    float n = norm();
    Quat ret = *this;
    ret.x_ /= n;
    ret.y_ /= n;
    ret.z_ /= n;
    ret.w_ /= n;
    return ret;
  }
  void normalize()
  {
    *this = normalized();
  }
  Quat conj() const
  {
    return Quat(-x_, -y_, -z_, w_);
  }
  Quat inv() const
  {
    return conj() / dot(*this);
  }
  Vec3 getRPY() const
  {
    const float ysq = y_ * y_;
    const float t0 = -2.0 * (ysq + z_ * z_) + 1.0;
    const float t1 = +2.0 * (x_ * y_ + w_ * z_);
    float t2 = -2.0 * (x_ * z_ - w_ * y_);
    const float t3 = +2.0 * (y_ * z_ + w_ * x_);
    const float t4 = -2.0 * (x_ * x_ + ysq) + 1.0;

    if (t2 > 1.0)
      t2 = 1.0;
    if (t2 < -1.0)
      t2 = -1.0;

    return Vec3(std::atan2(t3, t4), std::asin(t2), std::atan2(t1, t0));
  }
  void setRPY(const Vec3& rpy)
  {
    const float t2 = cos(rpy.x_ * 0.5);
    const float t3 = sin(rpy.x_ * 0.5);
    const float t4 = cos(rpy.y_ * 0.5);
    const float t5 = sin(rpy.y_ * 0.5);
    const float t0 = cos(rpy.z_ * 0.5);
    const float t1 = sin(rpy.z_ * 0.5);

    x_ = t0 * t3 * t4 - t1 * t2 * t5;
    y_ = t0 * t2 * t5 + t1 * t3 * t4;
    z_ = t1 * t2 * t4 - t0 * t3 * t5;
    w_ = t0 * t2 * t4 + t1 * t3 * t5;
  }
  void setAxisAng(const Vec3& axis, const float& ang)
  {
    const Vec3 a = axis / axis.norm();
    const float s = sinf(ang / 2.0);
    x_ = a.x_ * s;
    y_ = a.y_ * s;
    z_ = a.z_ * s;
    w_ = cosf(ang / 2.0);
    normalize();
  }
  void getAxisAng(Vec3& axis, float& ang) const
  {
    if (fabs(w_) >= 1.0 - 0.000001)
    {
      ang = 0.0;
      axis = Vec3(0.0, 0.0, 1.0);
      return;
    }
    ang = acosf(w_) * 2.0;
    if (ang > M_PI)
      ang -= 2.0 * M_PI;
    const float wsq = 1.0 - w_ * w_;
    axis = Vec3(x_, y_, z_) / sqrtf(wsq);
  }
  void rotateAxis(const Quat& r)
  {
    Vec3 axis;
    float ang;
    getAxisAng(axis, ang);
    setAxisAng(r * axis, ang);
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_QUAT_H

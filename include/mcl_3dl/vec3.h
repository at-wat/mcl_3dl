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

#ifndef MCL_3DL_VEC3_H
#define MCL_3DL_VEC3_H

#include <cmath>

namespace mcl_3dl
{
class Vec3
{
public:
  float x_;
  float y_;
  float z_;
  inline constexpr Vec3(const float x, const float y, const float z)
    : x_(x)
    , y_(y)
    , z_(z)
  {
  }
  inline constexpr Vec3()
    : x_(0)
    , y_(0)
    , z_(0)
  {
  }
  inline float& operator[](const size_t i)
  {
    switch (i)
    {
      case 0:
        return x_;
        break;
      case 1:
        return y_;
        break;
      case 2:
        return z_;
        break;
      default:
        break;
    }
    return x_;
  }
  inline float operator[](const size_t i) const
  {
    switch (i)
    {
      case 0:
        return x_;
        break;
      case 1:
        return y_;
        break;
      case 2:
        return z_;
        break;
      default:
        break;
    }
    return x_;
  }
  inline constexpr bool operator==(const Vec3& q) const
  {
    return x_ == q.x_ && y_ == q.y_ && z_ == q.z_;
  }
  inline constexpr bool operator!=(const Vec3& q) const
  {
    return !operator==(q);
  }
  inline constexpr Vec3 operator+(const Vec3& q) const
  {
    return Vec3(x_ + q.x_, y_ + q.y_, z_ + q.z_);
  }
  inline constexpr Vec3 operator-(const Vec3& q) const
  {
    return Vec3(x_ - q.x_, y_ - q.y_, z_ - q.z_);
  }
  inline constexpr Vec3 operator-() const
  {
    return Vec3(-x_, -y_, -z_);
  }
  inline constexpr Vec3 operator*(const float s) const
  {
    return Vec3(x_ * s, y_ * s, z_ * s);
  }
  inline constexpr Vec3 operator/(const float s) const
  {
    return Vec3(x_ / s, y_ / s, z_ / s);
  }
  inline Vec3& operator+=(const Vec3& q)
  {
    *this = *this + q;
    return *this;
  }
  inline Vec3& operator-=(const Vec3& q)
  {
    *this = *this - q;
    return *this;
  }
  inline Vec3& operator*=(const float& s)
  {
    *this = *this * s;
    return *this;
  }
  inline Vec3& operator/=(const float& s)
  {
    *this = *this / s;
    return *this;
  }
  inline constexpr float dot(const Vec3& q) const
  {
    return x_ * q.x_ + y_ * q.y_ + z_ * q.z_;
  }
  inline constexpr Vec3 cross(const Vec3& q) const
  {
    return Vec3(y_ * q.z_ - z_ * q.y_,
                z_ * q.x_ - x_ * q.z_,
                x_ * q.y_ - y_ * q.x_);
  }
  inline constexpr Vec3 times(const Vec3& q) const
  {
    return Vec3(x_ * q.x_, y_ * q.y_, z_ * q.z_);
  }
  inline float norm() const
  {
    return std::sqrt(dot(*this));
  }
  inline Vec3 normalized() const
  {
    return *this / norm();
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_VEC3_H

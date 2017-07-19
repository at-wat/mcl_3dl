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

#ifndef VEC3_H
#define VEC3_H

class Vec3
{
public:
  float x;
  float y;
  float z;
  Vec3(const float x, const float y, const float z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  Vec3()
  {
    x = y = z = 0.0;
  }
  float &operator[](const size_t i)
  {
    switch (i)
    {
      case 0:
        return x;
        break;
      case 1:
        return y;
        break;
      case 2:
        return z;
        break;
      default:
        break;
    }
    return x;
  }
  Vec3 operator+(const Vec3 &q) const
  {
    return Vec3(x + q.x, y + q.y, z + q.z);
  }
  Vec3 operator-(const Vec3 &q) const
  {
    return Vec3(x - q.x, y - q.y, z - q.z);
  }
  Vec3 operator-() const
  {
    return Vec3(-x, -y, -z);
  }
  Vec3 operator*(const float &s) const
  {
    return Vec3(x * s, y * s, z * s);
  }
  Vec3 operator/(const float &s) const
  {
    return Vec3(x / s, y / s, z / s);
  }
  Vec3 &operator+=(const Vec3 &q)
  {
    *this = *this + q;
    return *this;
  }
  Vec3 operator*(const Vec3 &q) const
  {
    return Vec3(x * q.x, y * q.y, z * q.z);
  }
  Vec3 &operator*=(const Vec3 &q)
  {
    *this = *this * q;
    return *this;
  }
  Vec3 &operator*=(const float &s)
  {
    *this = *this * s;
    return *this;
  }
  Vec3 &operator/=(const float &s)
  {
    *this = *this / s;
    return *this;
  }
  float dot(const Vec3 &q) const
  {
    return x * q.x + y * q.y + z * q.z;
  }
  Vec3 cross(const Vec3 &q) const
  {
    return Vec3(y * q.z - z * q.y,
                z * q.x - x * q.z,
                x * q.y - y * q.x);
  }
  float norm() const
  {
    return sqrtf(dot(*this));
  }
  Vec3 normalized() const
  {
    return *this / norm();
  }
};

#endif  // VEC3_H

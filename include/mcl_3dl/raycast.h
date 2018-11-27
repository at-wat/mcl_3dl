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

#ifndef MCL_3DL_RAYCAST_H
#define MCL_3DL_RAYCAST_H

#include <mcl_3dl/vec3.h>
#include <mcl_3dl/chunked_kdtree.h>

#include <vector>

namespace mcl_3dl
{
template <typename POINT_TYPE>
class Raycast
{
public:
  class CastResult
  {
  public:
    Vec3 pos_;
    bool collision_;
    float sin_angle_;

    CastResult(const Vec3 pos, bool collision, float sin_angle)
      : pos_(pos)
      , collision_(collision)
      , sin_angle_(sin_angle)
    {
    }
  };
  class Iterator
  {
  protected:
    typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree_;
    Vec3 pos_;
    Vec3 inc_;
    size_t count_;
    size_t length_;
    float grid_min_;
    float grid_max_;

  public:
    friend Raycast;

    Iterator(
        typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree,
        const Vec3 begin, const Vec3 end,
        const float grid_min, const float grid_max)
    {
      kdtree_ = kdtree;
      length_ = floorf((end - begin).norm() / grid_min - sqrtf(2.0));
      inc_ = (end - begin).normalized() * grid_min;
      pos_ = begin + inc_;
      count_ = 1;
      grid_min_ = grid_min;
      grid_max_ = grid_max;
    }
    Iterator& operator++()
    {
      ++count_;
      pos_ += inc_;
      return *this;
    }
    CastResult operator*()
    {
      bool collision(false);
      float sin_ang(0.0);

      POINT_TYPE center;
      center.x = pos_.x_;
      center.y = pos_.y_;
      center.z = pos_.z_;
      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      if (kdtree_->radiusSearch(
              center,
              sqrtf(2.0) * grid_max_ / 2.0, id, sqdist, 1))
      {
        collision = true;

        const float d0 = sqrtf(sqdist[0]);
        const Vec3 pos_prev = pos_ - (inc_ * 2.0);
        POINT_TYPE center_prev;
        center_prev.x = pos_prev.x_;
        center_prev.y = pos_prev.y_;
        center_prev.z = pos_prev.z_;
        if (kdtree_->radiusSearch(
                center_prev,
                grid_min_ * 2 + sqrtf(2.0) * grid_max_ / 2.0, id, sqdist, 1))
        {
          const float d1 = sqrtf(sqdist[0]);
          sin_ang = fabs(d1 - d0) / (grid_min_ * 2.0);
        }
        else
        {
          sin_ang = 1.0;
        }
      }
      return CastResult(pos_, collision, sin_ang);
    }
    bool operator!=(const Iterator& a) const
    {
      return count_ != a.count_;
    }
  };

protected:
  typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree_;
  Iterator begin_;
  Iterator end_;

public:
  Raycast(
      typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree,
      const Vec3 begin, const Vec3 end, const float grid, const float grid_max)
    : begin_(kdtree, begin, end, grid, grid_max)
    , end_(kdtree, begin, end, grid, grid_max)
  {
    kdtree_ = kdtree;
    end_.count_ = end_.length_;
  }
  Iterator begin()
  {
    return begin_;
  }
  Iterator end()
  {
    return end_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_RAYCAST_H

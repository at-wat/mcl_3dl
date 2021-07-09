/*
 * Copyright (c) 2020, the mcl_3dl authors
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

#ifndef MCL_3DL_RAYCASTS_RAYCAST_USING_KDTREE_H
#define MCL_3DL_RAYCASTS_RAYCAST_USING_KDTREE_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
template <typename POINT_TYPE>
class RaycastUsingKDTree : public Raycast<POINT_TYPE>
{
  using typename Raycast<POINT_TYPE>::CastResult;

public:
  RaycastUsingKDTree(const float map_grid_size_x, const float map_grid_size_y, const float map_grid_size_z,
                     const float hit_tolerance)
    : Raycast<POINT_TYPE>()
    , map_grid_min_(std::min({map_grid_size_x, map_grid_size_y, map_grid_size_z}))
    , map_grid_max_(std::max({map_grid_size_x, map_grid_size_y, map_grid_size_z}))
    , hit_tolerance_(hit_tolerance)
  {
  }

  void setRay(typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree, const Vec3 ray_begin, const Vec3 ray_end) final
  {
    kdtree_ = kdtree;
    length_ = std::floor(((ray_end - ray_begin).norm() + hit_tolerance_) / map_grid_min_);
    inc_ = (ray_end - ray_begin).normalized() * map_grid_min_;
    count_ = 1;
    pos_ = ray_begin + inc_;
  }

  bool getNextCastResult(CastResult& result) final
  {
    if (count_ >= length_)
    {
      return false;
    }
    bool collision(false);
    float sin_ang(0.0);

    const POINT_TYPE* point = nullptr;
    POINT_TYPE center;
    center.x = pos_.x_;
    center.y = pos_.y_;
    center.z = pos_.z_;
    std::vector<int> id(1);
    std::vector<float> sqdist(1);
    if (kdtree_->radiusSearch(center, std::sqrt(2.0) * map_grid_max_ / 2.0, id, sqdist, 1))
    {
      collision = true;
      point = &(kdtree_->getInputCloud()->points[id[0]]);

      const float d0 = std::sqrt(sqdist[0]);
      const Vec3 pos_prev = pos_ - (inc_ * 2.0);
      POINT_TYPE center_prev;
      center_prev.x = pos_prev.x_;
      center_prev.y = pos_prev.y_;
      center_prev.z = pos_prev.z_;
      if (kdtree_->radiusSearch(center_prev, map_grid_min_ * 2 + std::sqrt(2.0) * map_grid_max_ / 2.0, id, sqdist, 1))
      {
        const float d1 = std::sqrt(sqdist[0]);
        sin_ang = fabs(d1 - d0) / (map_grid_min_ * 2.0);
      }
      else
      {
        sin_ang = 1.0;
      }
    }
    result = CastResult(pos_, collision, sin_ang, point);

    ++count_;
    pos_ += inc_;
    return true;
  }

private:
  typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree_;
  Vec3 pos_;
  Vec3 inc_;
  int length_;
  int count_;
  const float map_grid_min_;
  const float map_grid_max_;
  const float hit_tolerance_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_RAYCASTS_RAYCAST_USING_KDTREE_H

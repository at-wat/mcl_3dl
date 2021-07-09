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

#ifndef MCL_3DL_RAYCASTS_RAYCAST_USING_DDA_H
#define MCL_3DL_RAYCASTS_RAYCAST_USING_DDA_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/vec3.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

namespace mcl_3dl
{
template <typename POINT_TYPE>
class RaycastUsingDDA : public Raycast<POINT_TYPE>
{
  using typename Raycast<POINT_TYPE>::CastResult;

public:
  RaycastUsingDDA(const double map_grid_size_x, const double map_grid_size_y, const double map_grid_size_z,
                  const double dda_grid_size, const double ray_angle_half, const double hit_tolerance)
    : Raycast<POINT_TYPE>()
    , min_dist_thr_sq_(std::pow(map_grid_size_x, 2) + std::pow(map_grid_size_y, 2) + std::pow(map_grid_size_y, 2))
    , dda_grid_size_(dda_grid_size)
    , ray_angle_half_(ray_angle_half)
    , hit_tolerance_(hit_tolerance)
  {
  }

  void setRay(typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree, const Vec3 ray_begin, const Vec3 ray_end_org) final
  {
    kdtree_ = kdtree;
    updatePointCloud();
    if (!isPointWithinMap(ray_begin))
    {
      max_movement_ = 0;
      pos_ = 0;
      return;
    }
    ray_begin_ = ray_begin;
    ray_direction_vector_ = (ray_end_org - ray_begin).normalized();
    const Vec3 ray_end = ray_end_org + ray_direction_vector_ * hit_tolerance_;
    begin_index_ = toIndex(ray_begin);
    end_index_ = toIndex(ray_end);
    const Eigen::Vector3i distance_index = end_index_ - begin_index_;
    pos_ = 0;
    max_movement_ = std::abs(distance_index[0]) + std::abs(distance_index[1]) + std::abs(distance_index[2]);
    step_ = Eigen::Vector3i((distance_index[0] < 0) ? -1 : 1, (distance_index[1] < 0) ? -1 : 1,
                            (distance_index[2] < 0) ? -1 : 1);

    current_index_ = begin_index_;
    for (int i = 0; i < 3; ++i)
    {
      if (distance_index[i] == 0)
      {
        initial_edges_[i] = std::numeric_limits<double>::infinity();
        t_delta_[i] = std::numeric_limits<double>::infinity();
      }
      else
      {
        const double nearest = (ray_direction_vector_[i] < 0) ? begin_index_[i] * dda_grid_size_ + min_p_[i] :
                                                                (begin_index_[i] + 1) * dda_grid_size_ + min_p_[i];
        initial_edges_[i] = std::abs((nearest - ray_begin[i]) / ray_direction_vector_[i]);
        t_delta_[i] = std::abs(dda_grid_size_ / ray_direction_vector_[i]);
      }
    }
    t_max_ = initial_edges_;
  }

  bool getNextCastResult(CastResult& result) final
  {
    ++pos_;
    if (pos_ >= max_movement_)
    {
      return false;
    }

    if (t_max_[0] < t_max_[1])
    {
      if (t_max_[0] < t_max_[2])
      {
        if (!incrementIndex(0))
        {
          return false;
        }
      }
      else
      {
        if (!incrementIndex(2))
        {
          return false;
        }
      }
    }
    else
    {
      if (t_max_[1] < t_max_[2])
      {
        if (!incrementIndex(1))
        {
          return false;
        }
      }
      else
      {
        if (!incrementIndex(2))
        {
          return false;
        }
      }
    }
    const POINT_TYPE* collided_point = hasIntersection(current_index_);
    if (collided_point)
    {
      // TODO(nhatao): Implement estimation of the angle of incidence.
      result = {fromIndex(current_index_), true, 1.0, collided_point};
    }
    else
    {
      result = {fromIndex(current_index_), false, 1.0, collided_point};
    }
    return true;
  }

private:
  void updatePointCloud()
  {
    if (!kdtree_->getInputCloud())
    {
      return;
    }
    if ((kdtree_->getInputCloud()->header.stamp == point_cloud_header_.stamp) && !points_.empty())
    {
      return;
    }
    const typename pcl::PointCloud<POINT_TYPE>::ConstPtr& input = kdtree_->getInputCloud();
    point_cloud_header_ = input->header;

    pcl::getMinMax3D(*input, min_p_, max_p_);
    int point_total = 1;
    for (int i = 0; i < 3; ++i)
    {
      map_size_[i] = static_cast<size_t>((max_p_[i] - min_p_[i]) / dda_grid_size_) + 1;
      point_total *= map_size_[i];
    }

    points_.clear();
    point_exists_.resize(point_total);
    std::fill(point_exists_.begin(), point_exists_.end(), 0);
    for (const auto& p : *input)
    {
      setExists(&p);
    }
  }

  bool incrementIndex(const int i)
  {
    current_index_[i] += step_[i];
    // Do not increment t_max to reduce calcuration errors
    t_max_[i] = initial_edges_[i] + t_delta_[i] * std::abs(current_index_[i] - begin_index_[i]);
    if (current_index_[i] < 0 || map_size_[i] <= current_index_[i])
    {
      pos_ = max_movement_;
      return false;
    }
    return true;
  }

  Eigen::Vector3i toIndex(const Vec3& point) const
  {
    return Eigen::Vector3i(static_cast<int>((point[0] - min_p_[0]) / dda_grid_size_),
                           static_cast<int>((point[1] - min_p_[1]) / dda_grid_size_),
                           static_cast<int>((point[2] - min_p_[2]) / dda_grid_size_));
  }

  Eigen::Vector3i toIndex(const POINT_TYPE* point) const
  {
    return Eigen::Vector3i(static_cast<int>((point->x - min_p_[0]) / dda_grid_size_),
                           static_cast<int>((point->y - min_p_[1]) / dda_grid_size_),
                           static_cast<int>((point->z - min_p_[2]) / dda_grid_size_));
  }

  Vec3 fromIndex(const Eigen::Vector3i& index) const
  {
    return Vec3((index[0] + 0.5) * dda_grid_size_ + min_p_[0], (index[1] + 0.5) * dda_grid_size_ + min_p_[1],
                (index[2] + 0.5) * dda_grid_size_ + min_p_[2]);
  }

  size_t getArrayIndex(const Eigen::Vector3i& cell) const
  {
    return static_cast<size_t>(cell[0] + cell[1] * map_size_[0] + cell[2] * (map_size_[0] * map_size_[1]));
  }

  void setExists(const POINT_TYPE* point)
  {
    const size_t array_index = getArrayIndex(toIndex(point));
    point_exists_[array_index] = 1;
    points_[array_index].push_back(point);
  }

  const POINT_TYPE* hasIntersection(const Eigen::Vector3i& target_cell) const
  {
    const size_t array_index = getArrayIndex(target_cell);
    if (!point_exists_[array_index])
    {
      return nullptr;
    }
    for (const POINT_TYPE* target_org : points_.find(array_index)->second)
    {
      const Vec3 target_rel(target_org->x - ray_begin_[0], target_org->y - ray_begin_[1],
                            target_org->z - ray_begin_[2]);
      const double dist_to_perpendicular_foot = std::abs(target_rel.dot(ray_direction_vector_));
      const double dist_threshold_sq =
          std::max(std::pow(ray_angle_half_ * dist_to_perpendicular_foot, 2), min_dist_thr_sq_);
      const double dist_sq = target_rel.dot(target_rel) - dist_to_perpendicular_foot * dist_to_perpendicular_foot;
      if (dist_sq < dist_threshold_sq)
      {
        return target_org;
      }
    }
    return nullptr;
  }

  bool isPointWithinMap(const Vec3& point) const
  {
    for (int i = 0; i < 3; ++i)
    {
      if ((point[i] < min_p_[i]) || (max_p_[i] < point[i]))
      {
        return false;
      }
    }
    return true;
  }

  typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree_;
  const double min_dist_thr_sq_;
  const double dda_grid_size_;
  const double ray_angle_half_;
  const double hit_tolerance_;

  Eigen::Vector3i map_size_;
  pcl::PCLHeader point_cloud_header_;

  std::vector<uint8_t> point_exists_;
  std::unordered_map<size_t, std::vector<const POINT_TYPE*>> points_;
  Eigen::Vector4f min_p_;
  Eigen::Vector4f max_p_;

  Eigen::Vector3i begin_index_;
  Eigen::Vector3i end_index_;
  Vec3 ray_begin_;
  Vec3 ray_direction_vector_;
  int max_movement_;
  int pos_;
  Vec3 t_max_;
  Vec3 t_delta_;
  Eigen::Vector3i current_index_;
  Vec3 initial_edges_;
  Eigen::Vector3i step_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_RAYCASTS_RAYCAST_USING_DDA_H

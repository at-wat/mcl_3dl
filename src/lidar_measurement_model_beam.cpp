/*
 * Copyright (c) 2018, the mcl_3dl authors
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/raycasts/raycast_using_dda.h>
#include <mcl_3dl/raycasts/raycast_using_kdtree.h>
#include <mcl_3dl/vec3.h>

#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>

namespace mcl_3dl
{
LidarMeasurementModelBeam::LidarMeasurementModelBeam(
    const float map_grid_x, const float map_grid_y, const float map_grid_z)
  : map_grid_x_(map_grid_x)
  , map_grid_y_(map_grid_y)
  , map_grid_z_(map_grid_z)
{
  search_range_ = std::max({map_grid_x_, map_grid_y_, map_grid_z_}) * 4;
}

void LidarMeasurementModelBeam::loadConfig(
    const ros::NodeHandle& nh,
    const std::string& name)
{
  ros::NodeHandle pnh(nh, name);

  int num_points, num_points_global;
  pnh.param("num_points", num_points, 3);
  pnh.param("num_points_global", num_points_global, 0);
  num_points_default_ = num_points_ = num_points;
  num_points_global_ = num_points_global;

  double clip_near, clip_far;
  pnh.param("clip_near", clip_near, 0.5);
  pnh.param("clip_far", clip_far, 4.0);
  clip_near_sq_ = clip_near * clip_near;
  clip_far_sq_ = clip_far * clip_far;

  double clip_z_min, clip_z_max;
  pnh.param("clip_z_min", clip_z_min, -2.0);
  pnh.param("clip_z_max", clip_z_max, 2.0);
  clip_z_min_ = clip_z_min;
  clip_z_max_ = clip_z_max;

  double beam_likelihood_min;
  pnh.param("beam_likelihood", beam_likelihood_min, 0.2);
  beam_likelihood_min_ = beam_likelihood_min;
  beam_likelihood_ = std::pow(beam_likelihood_min, 1.0 / static_cast<float>(num_points));

  double ang_total_ref;
  pnh.param("ang_total_ref", ang_total_ref, M_PI / 6.0);
  sin_total_ref_ = sinf(ang_total_ref);

  int filter_label_max;
  pnh.param("filter_label_max", filter_label_max, static_cast<int>(0xFFFFFFFF));
  filter_label_max_ = filter_label_max;

  pnh.param("add_penalty_short_only_mode", add_penalty_short_only_mode_, true);
  double hit_range;
  pnh.param("hit_range", hit_range, 0.3);
  hit_range_sq_ = std::pow(hit_range, 2);
  bool use_raycast_using_dda;
  pnh.param("use_raycast_using_dda", use_raycast_using_dda, false);
  if (use_raycast_using_dda)
  {
    double ray_angle_half;
    pnh.param("ray_angle_half", ray_angle_half, 0.25 * M_PI / 180.0);
    double dda_grid_size;
    pnh.param("dda_grid_size", dda_grid_size, 0.2);
    const double grid_size_max = std::max({map_grid_x_, map_grid_y_, map_grid_z_});
    if (dda_grid_size < grid_size_max)
    {
      ROS_WARN("dda_grid_size must be larger than grid size. New value: %f", grid_size_max);
      dda_grid_size = grid_size_max;
    }

    raycaster_ = std::make_shared<RaycastUsingDDA<PointType>>(
        map_grid_x_, map_grid_y_, map_grid_z_, dda_grid_size, ray_angle_half, hit_range);
  }
  else
  {
    raycaster_ = std::make_shared<RaycastUsingKDTree<PointType>>(
        map_grid_x_, map_grid_y_, map_grid_z_, hit_range);
  }
}

void LidarMeasurementModelBeam::setGlobalLocalizationStatus(
    const size_t num_particles,
    const size_t current_num_particles)
{
  if (current_num_particles <= num_particles)
  {
    num_points_ = num_points_default_;
    return;
  }
  size_t num = num_points_default_ * num_particles / current_num_particles;
  if (num < num_points_global_)
    num = num_points_global_;

  num_points_ = num;
}

typename pcl::PointCloud<LidarMeasurementModelBase::PointType>::Ptr
LidarMeasurementModelBeam::filter(
    const typename pcl::PointCloud<LidarMeasurementModelBase::PointType>::ConstPtr& pc) const
{
  const auto local_points_filter = [this](const LidarMeasurementModelBase::PointType& p)
  {
    if (p.x * p.x + p.y * p.y > clip_far_sq_)
      return true;
    if (p.x * p.x + p.y * p.y < clip_near_sq_)
      return true;
    if (p.z < clip_z_min_ || clip_z_max_ < p.z)
      return true;
    return false;
  };
  pcl::PointCloud<LidarMeasurementModelBase::PointType>::Ptr pc_filtered(
      new pcl::PointCloud<LidarMeasurementModelBase::PointType>);
  *pc_filtered = *pc;
  pc_filtered->erase(
      std::remove_if(pc_filtered->begin(), pc_filtered->end(), local_points_filter), pc_filtered->end());
  pc_filtered->width = 1;
  pc_filtered->height = pc_filtered->points.size();

  return sampler_->sample(pc_filtered, num_points_);
}

LidarMeasurementResult LidarMeasurementModelBeam::measure(
    typename ChunkedKdtree<LidarMeasurementModelBase::PointType>::Ptr& kdtree,
    const typename pcl::PointCloud<LidarMeasurementModelBase::PointType>::ConstPtr& pc,
    const std::vector<Vec3>& origins,
    const State6DOF& s) const
{
  if (!pc)
    return LidarMeasurementResult(1, 0);
  if (pc->size() == 0)
    return LidarMeasurementResult(1, 0);
  pcl::PointCloud<LidarMeasurementModelBase::PointType>::Ptr pc_particle(
      new pcl::PointCloud<LidarMeasurementModelBase::PointType>);

  float score_beam = 1.0;
  *pc_particle = *pc;
  s.transform(*pc_particle);
  for (auto& p : pc_particle->points)
  {
    const int beam_header_id = p.label;
    typename mcl_3dl::Raycast<PointType>::CastResult point;
    const BeamStatus status =
        getBeamStatus(kdtree, s.pos_ + s.rot_ * origins[beam_header_id], Vec3(p.x, p.y, p.z), point);
    if ((status == BeamStatus::SHORT) || (!add_penalty_short_only_mode_ && (status == BeamStatus::LONG)))
    {
      score_beam *= beam_likelihood_;
    }
  }
  if (score_beam < beam_likelihood_min_)
    score_beam = beam_likelihood_min_;

  return LidarMeasurementResult(score_beam, 1.0);
}

LidarMeasurementModelBeam::BeamStatus LidarMeasurementModelBeam::getBeamStatus(
    ChunkedKdtree<PointType>::Ptr& kdtree,
    const Vec3& lidar_pos,
    const Vec3& scan_pos,
    typename mcl_3dl::Raycast<PointType>::CastResult& result) const
{
  raycaster_->setRay(kdtree, lidar_pos, scan_pos);
  while (raycaster_->getNextCastResult(result))
  {
    if (!result.collision_)
      continue;
    if (result.point_->label > filter_label_max_)
      continue;
    // reject total reflection
    if (result.sin_angle_ > sin_total_ref_)
    {
      const float distance_from_point_sq = std::pow(scan_pos[0] - result.point_->x, 2) +
                                           std::pow(scan_pos[1] - result.point_->y, 2) +
                                           std::pow(scan_pos[2] - result.point_->z, 2);

      if (distance_from_point_sq < hit_range_sq_)
      {
        return BeamStatus::HIT;
      }
      else
      {
        return BeamStatus::SHORT;
      }
    }
    else
    {
      return BeamStatus::TOTAL_REFLECTION;
    }
  }
  return BeamStatus::LONG;
}

}  // namespace mcl_3dl

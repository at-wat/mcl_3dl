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
 * THIS SOFTWARE IS PROVIDEDNode BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
template <class STATE_TYPE, class POINT_TYPE>
class LidarMeasurementModelBeam : public LidarMeasurementModelBase<STATE_TYPE, POINT_TYPE>
{
  // POINT_TYPE must have label field (currently using intensity field as a label)
  static_assert(std::is_same<pcl::PointXYZI, POINT_TYPE>(), "Supported POINT_TYPE is PointXYZI");

private:
  size_t num_points_;
  size_t num_points_default_;
  size_t num_points_global_;
  float clip_far_sq_;
  float clip_near_sq_;
  float clip_z_min_;
  float clip_z_max_;
  float match_dist_min_;
  float match_dist_flat_;
  float beam_likelihood_min_;
  float beam_likelihood_;
  float sin_total_ref_;
  float map_grid_min_;
  float map_grid_max_;

  PointCloudRandomSampler<POINT_TYPE> sampler_;

public:
  void loadConfig(
      const ros::NodeHandle &nh,
      const std::string &name)
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

    double match_dist_min, match_dist_flat;
    pnh.param("match_dist_min", match_dist_min, 0.2);
    pnh.param("match_dist_flat", match_dist_flat, 0.05);
    match_dist_min_ = match_dist_min;
    match_dist_flat_ = match_dist_flat;

    double beam_likelihood_min;
    pnh.param("beam_likelihood", beam_likelihood_min, 0.2);
    beam_likelihood_min_ = beam_likelihood_min;
    beam_likelihood_ = powf(beam_likelihood_min, 1.0 / static_cast<float>(num_points));

    double ang_total_ref;
    pnh.param("ang_total_ref", ang_total_ref, M_PI / 6.0);
    sin_total_ref_ = sinf(ang_total_ref);
  }
  void setGlobalLocalizationStatus(
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
  LidarMeasurementModelBeam(const float x, const float y, const float z)
  {
    map_grid_min_ = std::min(std::min(x, y), z);
    map_grid_max_ = std::max(std::max(x, y), z);
  }

  typename pcl::PointCloud<POINT_TYPE>::Ptr filter(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr &pc) const
  {
    const auto local_points_filter = [this](const POINT_TYPE &p)
    {
      if (p.x * p.x + p.y * p.y > clip_far_sq_)
        return true;
      if (p.x * p.x + p.y * p.y < clip_near_sq_)
        return true;
      if (p.z < clip_z_min_ || clip_z_max_ < p.z)
        return true;
      if (p.intensity - roundf(p.intensity) > 0.01)
        return true;
      return false;
    };
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    *pc_filtered = *pc;
    pc_filtered->erase(
        std::remove_if(pc_filtered->begin(), pc_filtered->end(), local_points_filter), pc_filtered->end());
    pc_filtered->width = 1;
    pc_filtered->height = pc_filtered->points.size();

    return sampler_.sample(pc_filtered, num_points_);
  }

  std::pair<float, float> measure(
      typename mcl_3dl::ChunkedKdtree<POINT_TYPE>::Ptr &kdtree,
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr &pc,
      const std::vector<Vec3> &origins,
      const STATE_TYPE &s) const
  {
    if (!pc)
      return std::pair<float, float>(1, 0);
    if (pc->size() == 0)
      return std::pair<float, float>(1, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> id(1);
    std::vector<float> sqdist(1);

    float score_beam = 1.0;
    *pc_particle = *pc;
    s.transform(*pc_particle);
    for (auto &p : pc_particle->points)
    {
      const int beam_header_id = lroundf(p.intensity);
      Raycast<pcl::PointXYZI> ray(
          kdtree,
          s.pos + s.rot * origins[beam_header_id],
          Vec3(p.x, p.y, p.z),
          map_grid_min_, map_grid_max_);
      for (auto point : ray)
      {
        if (point.collision_)
        {
          // reject total reflection
          if (point.sin_angle_ > sin_total_ref_)
          {
            score_beam *= beam_likelihood_;
          }
          break;
        }
      }
    }
    if (score_beam < beam_likelihood_min_)
      score_beam = beam_likelihood_min_;

    return std::pair<float, float>(score_beam, 1.0);
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H

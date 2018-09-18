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

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_sampler.h>

namespace mcl_3dl
{
template <class STATE_TYPE, class POINT_TYPE>
class LidarMeasurementModelLikelihood : public LidarMeasurementModelBase<STATE_TYPE, POINT_TYPE>
{
private:
  size_t num_points_;
  size_t num_points_default_;
  size_t num_points_global_;
  float clip_far_sq_;
  float clip_near_sq_;
  float clip_z_min_;
  float clip_z_max_;
  float match_weight_;
  float match_dist_min_;
  float match_dist_flat_;

  PointCloudRandomSampler<POINT_TYPE> sampler_;

public:
  void loadConfig(
      const ros::NodeHandle &nh,
      const std::string &name)
  {
    ros::NodeHandle pnh(nh, name);

    int num_points, num_points_global;
    pnh.param("num_points", num_points, 96);
    pnh.param("num_points_global", num_points_global, 8);
    num_points_default_ = num_points_ = num_points;
    num_points_global_ = num_points_global;

    double clip_near, clip_far;
    pnh.param("clip_near", clip_near, 0.5);
    pnh.param("clip_far", clip_far, 10.0);
    clip_near_sq_ = clip_near * clip_near;
    clip_far_sq_ = clip_far * clip_far;

    double clip_z_min, clip_z_max;
    pnh.param("clip_z_min", clip_z_min, -2.0);
    pnh.param("clip_z_max", clip_z_max, 2.0);
    clip_z_min_ = clip_z_min;
    clip_z_max_ = clip_z_max;

    double match_weight;
    pnh.param("match_weight", match_weight, 5.0);
    match_weight_ = match_weight;

    double match_dist_min, match_dist_flat;
    pnh.param("match_dist_min", match_dist_min, 0.2);
    pnh.param("match_dist_flat", match_dist_flat, 0.05);
    match_dist_min_ = match_dist_min;
    match_dist_flat_ = match_dist_flat;
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
      const STATE_TYPE &s) const
  {
    if (!pc)
      return std::pair<float, float>(1, 0);
    if (pc->size() == 0)
      return std::pair<float, float>(1, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> id(1);
    std::vector<float> sqdist(1);

    float score_like = 0;
    *pc_particle = *pc;
    s.transform(*pc_particle);
    size_t num = 0;
    for (auto &p : pc_particle->points)
    {
      if (kdtree->radiusSearch(p, match_dist_min_, id, sqdist, 1))
      {
        const float dist = match_dist_min_ - std::max(sqrtf(sqdist[0]), match_dist_flat_);
        if (dist < 0.0)
          continue;

        score_like += dist * match_weight_;
        num++;
      }
    }
    const float match_ratio = static_cast<float>(num) / pc_particle->points.size();

    return std::pair<float, float>(score_like, match_ratio);
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H

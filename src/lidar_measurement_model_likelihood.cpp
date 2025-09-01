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

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/vec3.h>

#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>

namespace mcl_3dl
{

LidarMeasurementModelLikelihood::LidarMeasurementModelLikelihood(
    const std::shared_ptr<LidarMeasurementModelLikelihoodParameters>& params)
  : params_(params)
{
  refreshParameters();
}

void LidarMeasurementModelLikelihood::refreshParameters()
{
  num_points_ = params_->num_points_default_;
  clip_near_sq_ = params_->clip_near_ * params_->clip_near_;
  clip_far_sq_ = params_->clip_far_ * params_->clip_far_;
}

void LidarMeasurementModelLikelihood::setGlobalLocalizationStatus(
    const size_t num_particles,
    const size_t current_num_particles)
{
  if (current_num_particles <= num_particles)
  {
    num_points_ = params_->num_points_default_;
    return;
  }
  size_t num = params_->num_points_default_ * num_particles / current_num_particles;
  if (num < params_->num_points_global_)
    num = params_->num_points_global_;

  num_points_ = num;
}

typename pcl::PointCloud<LidarMeasurementModelBase::PointType>::Ptr
LidarMeasurementModelLikelihood::filter(
    const typename pcl::PointCloud<LidarMeasurementModelBase::PointType>::ConstPtr& pc,
    const PointCloudRandomSampler<PointType>& sampler) const
{
  const auto local_points_filter = [this](const LidarMeasurementModelBase::PointType& p)
  {
    if (p.x * p.x + p.y * p.y > clip_far_sq_)
      return true;
    if (p.x * p.x + p.y * p.y < clip_near_sq_)
      return true;
    if (p.z < params_->clip_z_min_ || params_->clip_z_max_ < p.z)
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

  return sampler.sample(pc_filtered, num_points_);
}

LidarMeasurementResult LidarMeasurementModelLikelihood::measure(
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
  std::vector<int> id(1);
  std::vector<float> sqdist(1);

  float score_like = 0;
  *pc_particle = *pc;
  s.transform(*pc_particle);
  size_t num = 0;
  for (auto& p : pc_particle->points)
  {
    if (kdtree->radiusSearch(p, params_->match_dist_min_, id, sqdist, 1))
    {
      const float dist = params_->match_dist_min_ - std::max(std::sqrt(sqdist[0]), params_->match_dist_flat_);
      if (dist < 0.0)
        continue;

      score_like += dist * params_->match_weight_;
      num++;
    }
  }
  const float match_ratio = static_cast<float>(num) / pc_particle->points.size();

  return LidarMeasurementResult(score_like, match_ratio);
}
}  // namespace mcl_3dl

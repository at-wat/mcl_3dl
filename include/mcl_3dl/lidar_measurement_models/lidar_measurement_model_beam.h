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

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/lidar_measurement_model_base.h>
#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class LidarMeasurementModelBeam : public LidarMeasurementModelBase
{
private:
  std::shared_ptr<Raycast<LidarMeasurementModelBase::PointType>> raycaster_;
  size_t num_points_;
  size_t num_points_default_;
  size_t num_points_global_;
  float clip_far_sq_;
  float clip_near_sq_;
  float clip_z_min_;
  float clip_z_max_;
  float beam_likelihood_min_;
  float beam_likelihood_;
  float sin_total_ref_;
  float map_grid_x_;
  float map_grid_y_;
  float map_grid_z_;
  float search_range_;
  uint32_t filter_label_max_;
  float additional_search_range_;
  float hit_range_sq_;
  bool add_penalty_short_only_mode_;

public:
  enum class BeamStatus
  {
    SHORT,
    HIT,
    LONG,
    TOTAL_REFLECTION
  };
  LidarMeasurementModelBeam(const float grid_size_x, const float grid_size_y, const float grid_size_z);

  inline float getMaxSearchRange() const
  {
    return search_range_;
  }
  inline float getSinTotalRef() const
  {
    return sin_total_ref_;
  }
  inline uint32_t getFilterLabelMax() const
  {
    return filter_label_max_;
  }
  void loadConfig(
      const ros::NodeHandle& nh,
      const std::string& name);
  void setGlobalLocalizationStatus(
      const size_t num_particles,
      const size_t current_num_particles);
  pcl::PointCloud<PointType>::Ptr filter(
      const pcl::PointCloud<PointType>::ConstPtr& pc) const;
  LidarMeasurementResult measure(
      ChunkedKdtree<PointType>::Ptr& kdtree,
      const pcl::PointCloud<PointType>::ConstPtr& pc,
      const std::vector<Vec3>& origins,
      const State6DOF& s) const;
  BeamStatus getBeamStatus(
      ChunkedKdtree<PointType>::Ptr& kdtree,
      const Vec3& beam_begin, const Vec3& beam_end,
      typename mcl_3dl::Raycast<PointType>::CastResult& result) const;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_BEAM_H

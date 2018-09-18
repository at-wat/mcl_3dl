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

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODEL_BASE_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODEL_BASE_H

#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
template <class STATE_TYPE, class POINT_TYPE>
class LidarMeasurementModelBase
{
  static_assert(
      std::is_base_of<pf::ParticleBase<float>, STATE_TYPE>::value,
      "STATE_TYPE is not based on pf::ParticleBase<float>");

public:
  using Ptr = std::shared_ptr<LidarMeasurementModelBase<STATE_TYPE, POINT_TYPE>>;

  virtual void loadConfig(
      const ros::NodeHandle &nh,
      const std::string &name) = 0;
  virtual void setGlobalLocalizationStatus(
      const size_t, const size_t) = 0;

  virtual typename pcl::PointCloud<POINT_TYPE>::Ptr filter(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr &) const = 0;

  virtual std::pair<float, float> measure(
      typename mcl_3dl::ChunkedKdtree<POINT_TYPE>::Ptr &,
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr &,
      const std::vector<Vec3> &,
      const STATE_TYPE &) const = 0;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODEL_BASE_H

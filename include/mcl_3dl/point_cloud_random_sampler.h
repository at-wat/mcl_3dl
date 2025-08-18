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

#ifndef MCL_3DL_POINT_CLOUD_RANDOM_SAMPLER_H
#define MCL_3DL_POINT_CLOUD_RANDOM_SAMPLER_H

#include <pcl/point_cloud.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
template <class POINT_TYPE>
class PointCloudRandomSampler
{
public:
  virtual void loadConfig(const ros::NodeHandle& nh) = 0;
  virtual typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr& pc, const size_t num) const = 0;
  virtual void setParticleStatistics(
    const State6DOF& mean, const std::vector<State6DOF>& covariances) = 0;

};

}  // namespace mcl_3dl

#endif  // MCL_3DL_POINT_CLOUD_RANDOM_SAMPLER_H

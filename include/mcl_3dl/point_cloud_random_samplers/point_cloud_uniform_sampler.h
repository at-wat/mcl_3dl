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

#ifndef MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_UNIFORM_SAMPLER_H
#define MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_UNIFORM_SAMPLER_H

#include <memory>
#include <random>

#include <pcl/point_cloud.h>

#include <mcl_3dl/point_cloud_random_sampler.h>

namespace mcl_3dl
{
template <class POINT_TYPE>
class PointCloudUniformSampler : public PointCloudRandomSampler<POINT_TYPE>
{
private:
  std::random_device seed_gen_;
  std::shared_ptr<std::default_random_engine> engine_;

public:
  PointCloudUniformSampler()
    : engine_(new std::default_random_engine(seed_gen_()))
  {
  }
  typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr& pc,
      const size_t num) const final
  {
    typename pcl::PointCloud<POINT_TYPE>::Ptr output(new pcl::PointCloud<POINT_TYPE>);
    output->header = pc->header;

    if (pc->points.size() == 0)
      return output;

    output->points.reserve(num);
    std::uniform_int_distribution<size_t> ud(0, pc->points.size() - 1);
    for (size_t i = 0; i < num; i++)
    {
      output->push_back(pc->points[ud(*engine_)]);
    }

    return output;
  }
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_UNIFORM_SAMPLER_H

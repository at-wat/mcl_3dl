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

#ifndef MCL_3DL_CLOUD_ACCUM_H
#define MCL_3DL_CLOUD_ACCUM_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

namespace mcl_3dl
{
class CloudAccumulationLogicBase
{
public:
  using Ptr = std::shared_ptr<CloudAccumulationLogicBase>;

  virtual void push(
      const std::string& key,
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      std::function<void()> process,
      std::function<bool(const sensor_msgs::PointCloud2::ConstPtr&)> accumulate,
      std::function<void()> clear) = 0;

  virtual void reset() = 0;
};

class CloudAccumulationLogicPassThrough : public CloudAccumulationLogicBase
{
public:
  void push(
      const std::string& key,
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      std::function<void()> process,
      std::function<bool(const sensor_msgs::PointCloud2::ConstPtr&)> accumulate,
      std::function<void()> clear) final;

  inline void reset() final
  {
  }
};

class CloudAccumulationLogic : public CloudAccumulationLogicBase
{
public:
  inline CloudAccumulationLogic(
      const size_t accum,
      const size_t accum_max)
    : accum_(accum)
    , accum_max_(accum_max)
    , cnt_accum_(0)
  {
  }

  void push(
      const std::string& key,
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      std::function<void()> process,
      std::function<bool(const sensor_msgs::PointCloud2::ConstPtr&)> accumulate,
      std::function<void()> clear) final;

  void reset() final;

private:
  size_t accum_;
  size_t accum_max_;
  size_t cnt_accum_;
  std::vector<std::string> keys_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_CLOUD_ACCUM_H

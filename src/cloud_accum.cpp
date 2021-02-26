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

#include <vector>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <mcl_3dl/cloud_accum.h>

namespace mcl_3dl
{
void CloudAccumulationLogicPassThrough::push(
    const std::string& key,
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    std::function<void()> process,
    std::function<bool(const sensor_msgs::PointCloud2::ConstPtr&)> accumulate,
    std::function<void()> clear)
{
  clear();
  if (accumulate(msg))
    process();
}

void CloudAccumulationLogic::push(
    const std::string& key,
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    std::function<void()> process,
    std::function<bool(const sensor_msgs::PointCloud2::ConstPtr&)> accumulate,
    std::function<void()> clear)
{
  // If total count of the accumulated cloud exceeds limit,
  // skip checking frame_id and force processing.
  if (keys_.size() < accum_max_)
  {
    if (keys_.size() == 0 || keys_.front() != key)
    {
      if (accumulate(msg))
      {
        if (keys_.size() == 0)
          cnt_accum_ = 1;

        keys_.push_back(key);
      }
      else
      {
        clear();
        reset();
      }
      return;
    }

    // Count number of clouds with the frame_id which was arrived
    // at first in this accumulation.
    if (cnt_accum_ < accum_)
    {
      if (accumulate(msg))
      {
        cnt_accum_++;
        keys_.push_back(key);
      }
      else
      {
        clear();
        reset();
      }
      return;
    }

    // Received (accum_cloud_ + 1) of clouds now.
    // Process already accumulated data and start next accumulation.
  }
  else
  {
    ROS_WARN(
        "Number of the accumulated cloud exceeds limit. "
        "Sensor with frame_id of %s may have been stopped.",
        keys_.front().c_str());
  }

  process();

  clear();
  reset();

  if (accumulate(msg))
  {
    keys_.push_back(key);
    cnt_accum_++;
  }
}

void CloudAccumulationLogic::reset()
{
  keys_.clear();
  cnt_accum_ = 0;
}
}  // namespace mcl_3dl

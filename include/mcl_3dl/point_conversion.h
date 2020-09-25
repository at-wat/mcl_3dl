/*
 * Copyright (c) 2019, the mcl_3dl authors
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

#ifndef MCL_3DL_POINT_CONVERSION_H
#define MCL_3DL_POINT_CONVERSION_H

#include <ros/ros.h>

#include <mcl_3dl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace mcl_3dl
{
namespace
{
template <typename PointTIn, typename PointTOut>
bool fromROSMsgImpl(
    const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointTOut>& pc)
{
  typename pcl::PointCloud<PointTIn>::Ptr raw(new typename pcl::PointCloud<PointTIn>);
  pcl::fromROSMsg(msg, *raw);
  if (raw->points.size() == 0)
  {
    ROS_ERROR("Given PointCloud2 is empty");
    return false;
  }
  pcl::copyPointCloud(*raw, pc);
  return true;
}
}  // namespace

template <typename PointT>
bool fromROSMsg(
    const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& pc)
{
  const int x_idx = getPointCloud2FieldIndex(msg, "x");
  const int y_idx = getPointCloud2FieldIndex(msg, "y");
  const int z_idx = getPointCloud2FieldIndex(msg, "z");
  const int intensity_idx = getPointCloud2FieldIndex(msg, "intensity");
  const int label_idx = getPointCloud2FieldIndex(msg, "label");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR("Given PointCloud2 doesn't have x, y, z fields");
    return false;
  }
  if (intensity_idx != -1)
  {
    if (label_idx != -1)
    {
      return fromROSMsgImpl<mcl_3dl::PointXYZIL, PointT>(msg, pc);
    }
    return fromROSMsgImpl<pcl::PointXYZI, PointT>(msg, pc);
  }
  if (label_idx != -1)
  {
    return fromROSMsgImpl<pcl::PointXYZL, PointT>(msg, pc);
  }
  return fromROSMsgImpl<pcl::PointXYZ, PointT>(msg, pc);
}
}  // namespace mcl_3dl
#endif  // MCL_3DL_POINT_CONVERSION_H

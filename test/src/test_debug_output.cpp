/*
 * Copyright (c) 2018-2019, the mcl_3dl authors
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

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace
{
void generateSamplePointcloud2(
    sensor_msgs::PointCloud2& cloud,
    const float offset_x,
    const float offset_y,
    const float offset_z)
{
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);

  class Point
  {
  public:
    float x_, y_, z_;
    Point(const float x, const float y, const float z)
      : x_(x)
      , y_(y)
      , z_(z)
    {
    }
  };
  std::vector<Point> points;
  // Draw cube
  for (float x = -1; x < 1; x += 0.05)
  {
    for (float y = -1; y < 1; y += 0.05)
    {
      points.push_back(Point(1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(-1.0 / 2 + offset_x, y + offset_y, x + offset_z));
    }
  }

  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const Point& p : points)
  {
    *iter_x = p.x_;
    *iter_y = p.y_;
    *iter_z = p.z_;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

inline sensor_msgs::PointCloud2 generateMapMsg(
    const float offset_x,
    const float offset_y,
    const float offset_z)
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, offset_x, offset_y, offset_z);
  cloud.header.frame_id = "map";
  return cloud;
}
inline sensor_msgs::PointCloud2 generateCloudMsg()
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, 0, 0, 0);
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = ros::Time::now();
  return cloud;
}
inline sensor_msgs::Imu generateImuMsg()
{
  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.header.stamp = ros::Time::now();
  imu.orientation.w = 1;
  imu.linear_acceleration.z = 9.8;
  return imu;
}
inline nav_msgs::Odometry generateOdomMsg()
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = 1;
  odom.pose.pose.orientation.w = 1;
  return odom;
}
}  // namespace

TEST(DebugOutput, MatchedUnmatched)
{
  sensor_msgs::PointCloud2::ConstPtr matched, unmatched;

  const boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> cb_matched =
      [&matched](const sensor_msgs::PointCloud2::ConstPtr& msg) -> void
  {
    matched = msg;
  };
  const boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> cb_unmatched =
      [&unmatched](const sensor_msgs::PointCloud2::ConstPtr& msg) -> void
  {
    unmatched = msg;
  };

  ros::NodeHandle nh("");
  ros::Subscriber sub_matched = nh.subscribe("mcl_3dl/matched", 1, cb_matched);
  ros::Subscriber sub_unmatched = nh.subscribe("mcl_3dl/unmatched", 1, cb_unmatched);

  ros::Publisher pub_mapcloud = nh.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);

  const float offset_x = 1;
  const float offset_y = 0;
  const float offset_z = 0;
  pub_mapcloud.publish(generateMapMsg(offset_x, offset_y, offset_z));

  ros::Duration(1.0).sleep();
  ros::Rate rate(10);
  for (int i = 0; i < 40; ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (matched && unmatched)
      break;
    pub_cloud.publish(generateCloudMsg());
    pub_imu.publish(generateImuMsg());
    pub_odom.publish(generateOdomMsg());
  }
  ASSERT_TRUE(ros::ok());

  ASSERT_TRUE(static_cast<bool>(matched));
  ASSERT_TRUE(static_cast<bool>(unmatched));

  ASSERT_GT(matched->width * matched->height, 0u);
  ASSERT_GT(unmatched->width * unmatched->height, 0u);

  {
    sensor_msgs::PointCloud2ConstIterator<float> x(*matched, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*matched, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*matched, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
      ASSERT_NEAR(*x, 0.5f, 0.1f);
      ASSERT_TRUE(-1.1 < *y && *y < 1.1);
      ASSERT_TRUE(-1.1 < *z && *z < 1.1);
    }
  }
  {
    sensor_msgs::PointCloud2ConstIterator<float> x(*unmatched, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*unmatched, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*unmatched, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
      ASSERT_NEAR(*x, -0.5f, 0.1f);
      ASSERT_TRUE(-1.1 < *y && *y < 1.1);
      ASSERT_TRUE(-1.1 < *z && *z < 1.1);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_debug_output");

  return RUN_ALL_TESTS();
}

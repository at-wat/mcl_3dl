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

#include <ros/ros.h>

#include <ros/master.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include <gtest/gtest.h>

namespace
{
void GenerateSinglePointPointcloud2(
    sensor_msgs::PointCloud2 &cloud,
    const float x, const float y, const float z)
{
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  modifier.resize(1);
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
}
void publishSinglePointPointcloud2(
    ros::Publisher &pub,
    const float x, const float y, const float z,
    const std::string frame_id,
    const ros::Time stamp)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  GenerateSinglePointPointcloud2(cloud, x, y, z);
  pub.publish(cloud);
}
}  // namespace

TEST(TransformFailure, NoDeadAgainstTransformFailure)
{
  ros::NodeHandle nh("");
  tf::TransformBroadcaster tfb;
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher pub_mapcloud = nh.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);

  ros::Rate rate(10);
  publishSinglePointPointcloud2(
      pub_mapcloud,
      0.0, 0.0, 0.0, "map", ros::Time::now());

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    ros::V_string nodes;
    ros::master::getNodes(nodes);
    bool found = false;
    for (auto node : nodes)
    {
      if (node == "/mcl_3dl")
      {
        found = true;
        break;
      }
    }
    if (found)
      break;
  }
  std::cerr << "mcl_3dl started" << std::endl;
  ros::Duration(1.0).sleep();
  int cnt = 0;
  // mcl_3dl is launched with required="true". This test is killed by roslaunch if mcl_3dl is dead.
  while (ros::ok())
  {
    ++cnt;
    tfb.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1)),
            ros::Time::now() + ros::Duration(0.1), "laser_link_base", "laser_link"));
    tfb.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1)),
            ros::Time::now() + ros::Duration(0.1), "map", "odom"));
    if (cnt > 10)
    {
      tfb.sendTransform(
          tf::StampedTransform(
              tf::Transform(tf::Quaternion(0, 0, 0, 1)),
              ros::Time::now() + ros::Duration(0.1), "base_link", "laser_link_base"));
    }
    if (cnt > 20)
    {
      tfb.sendTransform(
          tf::StampedTransform(
              tf::Transform(tf::Quaternion(0, 0, 0, 1)),
              ros::Time::now() + ros::Duration(0.1), "odom", "base_link"));
    }
    if (cnt > 30)
      break;

    publishSinglePointPointcloud2(
        pub_cloud,
        0.0, 0.0, 0.0, "laser_link", ros::Time::now());

    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_TRUE(ros::ok());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_transform_failure");

  return RUN_ALL_TESTS();
}

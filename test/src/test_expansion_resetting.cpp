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

#include <geometry_msgs/PoseArray.h>
#include <mcl_3dl_msgs/Status.h>
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
    sensor_msgs::PointCloud2 &cloud,
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
      points.push_back(Point(x / 2 + offset_x, y + offset_y, 1.0 + offset_z));
      points.push_back(Point(x / 2 + offset_x, y + offset_y, -1.0 + offset_z));
      points.push_back(Point(1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(-1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(x / 2 + offset_x, 1.0 + offset_y, y + offset_z));
      points.push_back(Point(x / 2 + offset_x, -1.0 + offset_y, y + offset_z));
    }
  }

  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const Point &p : points)
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

TEST(ExpansionResetting, ExpandAndResume)
{
  geometry_msgs::PoseArray::ConstPtr poses;
  mcl_3dl_msgs::Status::ConstPtr status;

  const boost::function<void(const geometry_msgs::PoseArray::ConstPtr &)> cb_pose =
      [&poses](const geometry_msgs::PoseArray::ConstPtr &msg) -> void
  {
    poses = msg;
  };
  const boost::function<void(const mcl_3dl_msgs::Status::ConstPtr &)> cb_status =
      [&status](const mcl_3dl_msgs::Status::ConstPtr &msg) -> void
  {
    status = msg;
  };

  ros::NodeHandle nh("");
  ros::Subscriber sub_pose = nh.subscribe("mcl_3dl/particles", 1, cb_pose);
  ros::Subscriber sub_status = nh.subscribe("mcl_3dl/status", 1, cb_status);

  ros::ServiceClient src_expansion_resetting =
      nh.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "expansion_resetting");

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
  // Wait until finishing expansion resetting
  for (int i = 0; i < 40; ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (status && status->status == mcl_3dl_msgs::Status::EXPANSION_RESETTING)
      i = 0;
    pub_cloud.publish(generateCloudMsg());
    pub_imu.publish(generateImuMsg());
    pub_odom.publish(generateOdomMsg());
  }
  ASSERT_TRUE(ros::ok());

  ASSERT_TRUE(static_cast<bool>(status));
  ASSERT_TRUE(static_cast<bool>(poses));

  const tf2::Transform true_pose(
      tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset_x, offset_y, offset_z));
  bool found_true_positive(false);
  for (const auto &pose : poses->poses)
  {
    tf2::Transform particle_pose;
    tf2::fromMsg(pose, particle_pose);

    const tf2::Transform tf_diff = particle_pose.inverse() * true_pose;
    if (tf_diff.getOrigin().length() < 2e-1 &&
        fabs(tf2::getYaw(tf_diff.getRotation())) < 2e-1)
      found_true_positive = true;
  }
  ASSERT_TRUE(found_true_positive);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_expansion_resetting");

  return RUN_ALL_TESTS();
}

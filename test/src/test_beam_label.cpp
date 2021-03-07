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

#include <random>
#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

namespace
{
void generateSamplePointcloud2(
    sensor_msgs::PointCloud2& cloud,
    const float offset_x,
    const float offset_y,
    const float offset_z,
    const float range)
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
      "label", 1, sensor_msgs::PointField::UINT32);

  const float resolution = 0.05;

  std::vector<Eigen::Vector4d> points;
  // Floor
  for (float x = -range; x < range; x += resolution)
  {
    for (float y = -range; y < range; y += resolution)
    {
      points.emplace_back(x, y, -1.0, 0);
    }
  }
  // Semi-transparent wall
  for (float x = -0.5; x < 0.5; x += resolution)
  {
    for (float z = -1.0; z < 0.0; z += resolution)
    {
      points.emplace_back(x, 1.0, z, 2);
    }
  }

  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_label(cloud, "label");

  for (const Eigen::Vector4d& p : points)
  {
    *iter_x = p[0] + offset_x;
    *iter_y = p[1] + offset_y;
    *iter_z = p[2] + offset_z;
    *iter_label = static_cast<uint32_t>(p[3]);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_label;
  }
}

inline sensor_msgs::PointCloud2 generateMapMsg(
    const float offset_x,
    const float offset_y,
    const float offset_z)
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, offset_x, offset_y, offset_z, 5.0);
  cloud.header.frame_id = "map";
  return cloud;
}
inline sensor_msgs::PointCloud2 generateCloudMsg()
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, 0, 0, 0, 2.0);
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
inline geometry_msgs::PoseWithCovarianceStamped generateInitialPose()
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[6 * 0 + 0] = std::pow(0.2, 2);
  pose.pose.covariance[6 * 1 + 1] = std::pow(0.2, 2);
  pose.pose.covariance[6 * 2 + 2] = std::pow(0.2, 2);
  pose.pose.covariance[6 * 3 + 3] = 0.0;
  pose.pose.covariance[6 * 4 + 4] = 0.0;
  pose.pose.covariance[6 * 5 + 5] = std::pow(0.05, 2);
  return pose;
}
}  // namespace

class BeamLabel : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pose_cov_;
  ros::Publisher pub_mapcloud_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_init_;

  geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose_cov_;

  void cbPoseCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    pose_cov_ = msg;
  }

  void SetUp()
  {
    pub_init_.publish(generateInitialPose());
    ros::Rate wait(10);
    for (int i = 0; i < 100; i++)
    {
      wait.sleep();
      ros::spinOnce();
      if (pose_cov_)
        break;
      if (!ros::ok())
        break;
    }
  }

public:
  BeamLabel()
  {
    sub_pose_cov_ = nh_.subscribe("amcl_pose", 1, &BeamLabel::cbPoseCov, this);

    pub_mapcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    pub_init_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
  }
};

TEST_F(BeamLabel, SemiTransparentWall)
{
  const float offset_x = 0.05;
  const float offset_y = 0.05;
  const float offset_z = 0;
  pub_mapcloud_.publish(generateMapMsg(offset_x, offset_y, offset_z));

  ros::Duration(1.0).sleep();
  ros::Rate rate(10);
  for (int i = 0; i < 100; ++i)
  {
    rate.sleep();
    ros::spinOnce();
    pub_cloud_.publish(generateCloudMsg());
    pub_imu_.publish(generateImuMsg());
    pub_odom_.publish(generateOdomMsg());
  }
  ASSERT_TRUE(ros::ok());

  ASSERT_TRUE(static_cast<bool>(pose_cov_));

  ASSERT_NEAR(pose_cov_->pose.pose.position.x, offset_x, 0.1);
  ASSERT_NEAR(pose_cov_->pose.pose.position.y, offset_y, 0.1);
  ASSERT_NEAR(pose_cov_->pose.pose.position.z, offset_z, 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_beam_label");
  ros::NodeHandle nh;  // workaround to keep the test node during the process life time

  return RUN_ALL_TESTS();
}

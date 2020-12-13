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

#include <limits>
#include <random>
#include <vector>

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

#include <gtest/gtest.h>

namespace
{
void generateSamplePointcloud2(
    sensor_msgs::PointCloud2& cloud,
    const float x0,
    const float y0,
    const float x1,
    const float y1,
    const float offset_x,
    const float offset_y,
    const float offset_z,
    const float offset_yaw)
{
  std::random_device seed;
  std::default_random_engine engine(seed());
  std::normal_distribution<float> rand(0, 0.01);

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
  const float grid_xy = 0.15;
  const float grid_z = 0.1;
  const float floor_size = 6.0;
  // Draw floor
  for (float x = -floor_size; x < floor_size; x += grid_xy)
    for (float y = -floor_size; y < floor_size; y += grid_xy)
      if (x0 < x && x < x1 && y0 < y && y < y1)
        points.push_back(Point(x, y, 0.0));
  // Draw objects
  for (float x = -2; x < 1.6; x += grid_xy)
    if (x0 < x && x < x1)
      for (float z = 0.5; z < 1.5; z += grid_z)
        points.push_back(Point(x, 1.6, z));
  for (float y = -0.5; y < 1.6; y += grid_xy)
    if (y0 < y && y < y1)
      for (float z = 0.5; z < 2.0; z += grid_z)
        points.push_back(Point(1.6, y, z));
  for (float x = 0; x < 1.6; x += grid_xy)
    if (x0 < x && x < x1)
      for (float z = 0.5; z < 2.0; z += grid_z)
        points.push_back(Point(x, -2.1 + x, z));

  const float o_cos = cosf(offset_yaw);
  const float o_sin = sinf(offset_yaw);
  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const Point& p : points)
  {
    *iter_x = p.x_ * o_cos - p.y_ * o_sin + offset_x + rand(engine);
    *iter_y = p.x_ * o_sin + p.y_ * o_cos + offset_y + rand(engine);
    *iter_z = p.z_ + offset_z + rand(engine);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

inline sensor_msgs::PointCloud2 generateMapMsg()
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, -100, -100, 100, 100, 0, 0, 0, 0);
  cloud.header.frame_id = "map";
  return cloud;
}
inline sensor_msgs::PointCloud2 generateCloudMsg(
    const float offset_x,
    const float offset_y,
    const float offset_z,
    const float offset_yaw)
{
  sensor_msgs::PointCloud2 cloud;
  generateSamplePointcloud2(
      cloud, -2, -2, 2, 2,
      offset_x, offset_y, offset_z, offset_yaw);
  cloud.header.frame_id = "laser";
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
inline nav_msgs::Odometry generateOdomMsg(const float x)
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = 5;
  odom.pose.pose.orientation.w = 1;
  return odom;
}
}  // namespace

class GlobalLocalization : public ::testing::TestWithParam<float>
{
};

INSTANTIATE_TEST_CASE_P(
    OdometryOffset, GlobalLocalization,
    ::testing::Values(5.0, 100.0));

TEST_P(GlobalLocalization, Localize)
{
  geometry_msgs::PoseArray::ConstPtr poses;
  mcl_3dl_msgs::Status::ConstPtr status;

  const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)> cb_pose =
      [&poses](const geometry_msgs::PoseArray::ConstPtr& msg) -> void
  {
    poses = msg;
  };
  const boost::function<void(const mcl_3dl_msgs::Status::ConstPtr&)> cb_status =
      [&status](const mcl_3dl_msgs::Status::ConstPtr& msg) -> void
  {
    status = msg;
  };

  ros::NodeHandle nh("");
  ros::Subscriber sub_pose = nh.subscribe("mcl_3dl/particles", 1, cb_pose);
  ros::Subscriber sub_status = nh.subscribe("mcl_3dl/status", 1, cb_status);

  ros::ServiceClient src_global_localization =
      nh.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "global_localization");

  ros::Publisher pub_mapcloud = nh.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);

  pub_mapcloud.publish(generateMapMsg());
  ros::Rate wait(10);
  for (int i = 0; i < 100; i++)
  {
    wait.sleep();
    ros::spinOnce();
    if (poses)
      break;
    ASSERT_TRUE(ros::ok());
  }

  for (float offset_x = -0.5; offset_x <= 0.51; offset_x += 1.0)
  {
    for (float offset_yaw = -M_PI / 2; offset_yaw <= M_PI / 2 + 0.1; offset_yaw += M_PI)
    {
      const float laser_frame_height = 0.5;
      const float offset_y = 0.54;
      const float offset_z = 0.0;

      ros::Rate rate(10);
      // Wait until mcl_3dl initialization
      while (ros::ok())
      {
        rate.sleep();
        ros::spinOnce();
        if (status && status->status == mcl_3dl_msgs::Status::NORMAL)
          break;
        pub_cloud.publish(
            generateCloudMsg(offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu.publish(generateImuMsg());
        pub_odom.publish(generateOdomMsg(0.0));
      }
      ASSERT_TRUE(ros::ok());
      for (int i = 0; i < 5; ++i)
      {
        // Publish odom multiple times to make mcl_3dl internal odometry diff zero
        pub_odom.publish(generateOdomMsg(GetParam()));
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }

      std_srvs::Trigger trigger;
      ASSERT_TRUE(src_global_localization.call(trigger));
      ros::spinOnce();

      // Wait until starting global localization
      while (ros::ok())
      {
        rate.sleep();
        ros::spinOnce();
        if (status && status->status == mcl_3dl_msgs::Status::GLOBAL_LOCALIZATION)
          break;
        pub_cloud.publish(
            generateCloudMsg(offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu.publish(generateImuMsg());
        pub_odom.publish(generateOdomMsg(GetParam()));
      }
      ASSERT_TRUE(ros::ok());

      // Wait until finishing global localization
      while (ros::ok())
      {
        rate.sleep();
        ros::spinOnce();
        if (status && status->status == mcl_3dl_msgs::Status::NORMAL)
          break;
        pub_cloud.publish(
            generateCloudMsg(offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu.publish(generateImuMsg());
        pub_odom.publish(generateOdomMsg(GetParam()));
      }
      ASSERT_TRUE(ros::ok());

      // Wait to improve accuracy
      for (int i = 0; i < 40; ++i)
      {
        rate.sleep();
        ros::spinOnce();
        pub_cloud.publish(
            generateCloudMsg(offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu.publish(generateImuMsg());
        pub_odom.publish(generateOdomMsg(GetParam()));
      }
      ASSERT_TRUE(ros::ok());

      ASSERT_TRUE(static_cast<bool>(status));
      ASSERT_TRUE(static_cast<bool>(poses));

      const tf2::Transform true_pose(
          tf2::Quaternion(0, 0, sinf(-offset_yaw / 2), cosf(-offset_yaw / 2)),
          tf2::Vector3(
              -(offset_x * cos(-offset_yaw) - offset_y * sin(-offset_yaw)),
              -(offset_x * sin(-offset_yaw) + offset_y * cos(-offset_yaw)),
              -offset_z));
      bool found_true_positive(false);
      float dist_err_min = std::numeric_limits<float>::max();
      float ang_err_min = std::numeric_limits<float>::max();
      for (const auto& pose : poses->poses)
      {
        tf2::Transform particle_pose;
        tf2::fromMsg(pose, particle_pose);

        const tf2::Transform tf_diff = particle_pose.inverse() * true_pose;
        const float dist_err = tf_diff.getOrigin().length();
        const float ang_err = fabs(tf2::getYaw(tf_diff.getRotation()));
        if (dist_err < 2e-1 && ang_err < 2e-1)
          found_true_positive = true;

        if (dist_err_min > dist_err)
        {
          dist_err_min = dist_err;
          ang_err_min = ang_err;
        }
      }
      ASSERT_TRUE(found_true_positive)
          << "Minimum position error: " << dist_err_min << std::endl
          << "Angular error: " << ang_err_min << std::endl;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_global_localization");
  ros::NodeHandle nh;  // workaround to keep the test node during the process life time

  return RUN_ALL_TESTS();
}

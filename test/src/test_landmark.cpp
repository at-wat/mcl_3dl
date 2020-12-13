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

#include <limits>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gtest/gtest.h>

namespace
{
inline geometry_msgs::PoseWithCovarianceStamped generatePoseWithCov(
    const float y, const float y_var, const float var_measure = 0.0)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position.y = y;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[6 * 0 + 0] = var_measure;
  pose.pose.covariance[6 * 1 + 1] = y_var;
  pose.pose.covariance[6 * 2 + 2] = var_measure;
  pose.pose.covariance[6 * 3 + 3] = var_measure;
  pose.pose.covariance[6 * 4 + 4] = var_measure;
  pose.pose.covariance[6 * 5 + 5] = var_measure;
  return pose;
}
std::pair<float, float> getMean(const std::vector<geometry_msgs::Pose>& poses)
{
  float mean = 0;
  for (const geometry_msgs::Pose p : poses)
  {
    mean += p.position.y;
  }
  mean /= poses.size();

  float root_mean = 0;
  for (const geometry_msgs::Pose p : poses)
  {
    root_mean += std::pow(p.position.y - mean, 2);
  }
  root_mean /= poses.size();

  return std::pair<float, float>(mean, root_mean);
}
}  // namespace

TEST(Landmark, Measurement)
{
  geometry_msgs::PoseArray::ConstPtr poses;

  const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)> cb_pose =
      [&poses](const geometry_msgs::PoseArray::ConstPtr& msg) -> void
  {
    poses = msg;
  };

  ros::NodeHandle nh("");
  ros::Subscriber sub_pose = nh.subscribe("mcl_3dl/particles", 1, cb_pose);
  ros::Publisher pub_init =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
  ros::Publisher pub_landmark =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_measurement", 1, true);

  ros::Duration(1.0).sleep();
  pub_init.publish(generatePoseWithCov(2.0, 1.0));

  ros::Rate wait(10);
  for (int i = 0; i < 100; i++)
  {
    wait.sleep();
    ros::spinOnce();
    if (poses)
      break;
    ASSERT_TRUE(ros::ok());
  }
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_TRUE(static_cast<bool>(poses));
  for (const geometry_msgs::Pose p : poses->poses)
  {
    ASSERT_FLOAT_EQ(p.position.x, 0.0f);
    ASSERT_FLOAT_EQ(p.position.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.x, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.y, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.w, 1.0f);
  }
  const std::pair<float, float> mean_init = getMean(poses->poses);
  ASSERT_NEAR(mean_init.first, 2.0f, 0.1f);
  ASSERT_NEAR(mean_init.second, 1.0f, 0.1f);

  poses = nullptr;
  pub_landmark.publish(generatePoseWithCov(2.6, 1.0, 1000.0 * 1000.0));
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_TRUE(static_cast<bool>(poses));
  for (const geometry_msgs::Pose p : poses->poses)
  {
    ASSERT_FLOAT_EQ(p.position.x, 0.0f);
    ASSERT_FLOAT_EQ(p.position.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.x, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.y, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.w, 1.0f);
  }
  const std::pair<float, float> mean_measured = getMean(poses->poses);
  ASSERT_NEAR(mean_measured.first, 2.3f, 0.1f);
  ASSERT_NEAR(mean_measured.second, 0.5f, 0.1f);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_landmark");

  return RUN_ALL_TESTS();
}

/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gtest/gtest.h>

TEST(ComparePose, Compare)
{
  ros::NodeHandle nh("~");

  nav_msgs::Path path;
  size_t i_path;

  double error_limit;
  nh.param("error_limit", error_limit, 0.3);

  const boost::function<void(const nav_msgs::Path::ConstPtr&)> cb_path =
      [&path, &i_path](const nav_msgs::Path::ConstPtr& msg) -> void
  {
    path = *msg;
    i_path = 0;
    fprintf(stderr, "compare_pose: reference received\n");
  };
  const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb_pose =
      [&path, &i_path, &error_limit](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) -> void
  {
    if (path.poses.size() > 0 && path.poses[i_path].header.stamp < ros::Time::now())
    {
      const float x_error = path.poses[i_path].pose.position.x - msg->pose.pose.position.x;
      const float y_error = path.poses[i_path].pose.position.y - msg->pose.pose.position.y;
      const float z_error = path.poses[i_path].pose.position.z - msg->pose.pose.position.z;
      const float error = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2) + std::pow(z_error, 2));
      const float x_sigma = std::sqrt(msg->pose.covariance[0 * 6 + 0]);
      const float y_sigma = std::sqrt(msg->pose.covariance[1 * 6 + 1]);
      const float z_sigma = std::sqrt(msg->pose.covariance[2 * 6 + 2]);

      fprintf(stderr, "compare_pose[%lu/%lu]:\n",
              i_path, path.poses.size());
      fprintf(stderr, "  position error/limit=%0.3f/%0.3f\n", error, error_limit);
      fprintf(stderr, "  x error/3sigma=%0.3f/%0.3f\n", x_error, x_sigma * 3.0);
      fprintf(stderr, "  y error/3sigma=%0.3f/%0.3f\n", y_error, y_sigma * 3.0);
      fprintf(stderr, "  z error/3sigma=%0.3f/%0.3f\n", z_error, z_sigma * 3.0);

      i_path++;
      if (i_path >= path.poses.size())
        ros::shutdown();

      ASSERT_FALSE(error > error_limit)
          << "Position error is larger then expected.";
      ASSERT_FALSE(fabs(x_error) > x_sigma * 3.0)
          << "Estimated variance is too small to continue tracking. (x)";
      ASSERT_FALSE(fabs(y_error) > y_sigma * 3.0)
          << "Estimated variance is too small to continue tracking. (y)";
      ASSERT_FALSE(fabs(z_error) > z_sigma * 3.0)
          << "Estimated variance is too small to continue tracking. (z)";
    }
  };

  ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 1, cb_pose);
  ros::Subscriber sub_path = nh.subscribe("poses_ref", 1, cb_path);

  ros::Rate wait(10);

  while (ros::ok())
  {
    ros::spinOnce();
    wait.sleep();
  }
  fprintf(stderr, "compare_pose finished\n");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "compare_pose");

  return RUN_ALL_TESTS();
}

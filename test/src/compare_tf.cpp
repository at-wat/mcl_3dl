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
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <gtest/gtest.h>

TEST(CompareTf, Compare)
{
  ros::NodeHandle nh("~");
  tf2_ros::Buffer tfbuf;
  tf2_ros::TransformListener tfl(tfbuf);

  int cnt = 0;
  int cnt_max;
  nh.param("cnt_max", cnt_max, 10);
  size_t tf_ex_cnt = 0;

  const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb_pose =
      [&tfbuf, &cnt, &cnt_max, &tf_ex_cnt](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) -> void
  {
    geometry_msgs::PoseStamped pose;
    try
    {
      geometry_msgs::PoseStamped pose_bl;
      pose_bl.header.frame_id = "base_link";
      pose_bl.header.stamp = msg->header.stamp;
      pose_bl.pose.orientation.w = 1.0;
      geometry_msgs::TransformStamped trans =
          tfbuf.lookupTransform("map", pose_bl.header.frame_id, pose_bl.header.stamp, ros::Duration(0.1));
      tf2::doTransform(pose_bl, pose, trans);
    }
    catch (tf2::TransformException& e)
    {
      tf_ex_cnt++;
      return;
    }
    const float x_error = pose.pose.position.x - msg->pose.pose.position.x;
    const float y_error = pose.pose.position.y - msg->pose.pose.position.y;
    const float z_error = pose.pose.position.z - msg->pose.pose.position.z;
    const float error = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2) + std::pow(z_error, 2));

    fprintf(stderr, "compare_tf[%d/%d]:\n", cnt, cnt_max);
    fprintf(stderr, "  error=%0.3f\n", error);

    cnt++;
    if (cnt >= cnt_max)
      ros::shutdown();

    ASSERT_FALSE(error > 0.05)
        << "tf output diverges from amcl_pose.";
  };
  ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 1, cb_pose);

  ros::Rate wait(1);

  while (ros::ok())
  {
    ros::spinOnce();
    wait.sleep();
  }

  ASSERT_FALSE(tf_ex_cnt > 1)
      << "tf exception occures more than once.";

  fprintf(stderr, "compare_tf finished\n");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "compare_tf");

  return RUN_ALL_TESTS();
}

/*
 * Copyright (c) 2016-2025, the mcl_3dl authors
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

#include <thread>  // NOLINT(build/c++11)

#include <gtest/gtest.h>

#include <dynamic_reconfigure/client.h>
#include <ros/ros.h>

#include <mcl_3dl/MCL3DLParamsConfig.h>
#include <mcl_3dl/parameters.h>

TEST(Parameters, DynamicParameters)
{
  ros::NodeHandle nh("/mcl_3dl");
  nh.setParam("std_warn_thresh_xy", 0.1);
  nh.setParam("std_warn_thresh_z", 0.2);
  nh.setParam("std_warn_thresh_yaw", 0.3);

  mcl_3dl::Parameters mcl_3dl_params;
  mcl_3dl_params.load(nh);

  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[0], 0.1);
  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[1], 0.2);
  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[2], 0.3);

  std::thread t(
      []()
      {
        dynamic_reconfigure::Client<mcl_3dl::MCL3DLParamsConfig> dynamic_reconfigure_client("/mcl_3dl");
        mcl_3dl::MCL3DLParamsConfig config;
        config.std_warn_thresh_xy = 0.5;
        config.std_warn_thresh_z = 0.6;
        config.std_warn_thresh_yaw = 0.7;

        while (ros::ok())
        {
          // Wait until parameter server becomes ready
          mcl_3dl::MCL3DLParamsConfig dummy;
          if (dynamic_reconfigure_client.getCurrentConfiguration(dummy, ros::Duration(0.1)))
          {
            break;
          }
        }
        if (!dynamic_reconfigure_client.setConfiguration(config))
        {
          FAIL();
        }
      });  // NOLINT(whitespace/braces)

  const ros::Time deadline = ros::Time::now() + ros::Duration(0.5);
  const ros::Duration wait(0.1);
  while (ros::ok() && ros::Time::now() < deadline)
  {
    ros::spinOnce();
    wait.sleep();
  }
  t.join();

  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[0], 0.5);
  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[1], 0.6);
  ASSERT_FLOAT_EQ(mcl_3dl_params.std_warn_thresh_[2], 0.7);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_parameters");

  return RUN_ALL_TESTS();
}

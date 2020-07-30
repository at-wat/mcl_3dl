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

#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <gtest/gtest.h>

#define UNDEF_COMPATIBILITY_LEVEL

namespace mcl_3dl_compat
{
int current_level;
int supported_level;
int default_level;
}  // namespace mcl_3dl_compat
#include <mcl_3dl_compat/compatibility.h>

TEST(Mcl3DlCompat, ParamRename)
{
  ros::NodeHandle nh;

  nh.setParam("param1", 1.0);
  nh.setParam("param2", 2.0);
  nh.setParam("param2_new", 3.0);
  nh.setParam("param3_new", 4.0);

  mcl_3dl_compat::paramRename<double>(nh, "param1_new", "param1");
  mcl_3dl_compat::paramRename<double>(nh, "param2_new", "param2");

  ASSERT_TRUE(nh.hasParam("param1_new"));
  ASSERT_TRUE(nh.hasParam("param2_new"));
  ASSERT_TRUE(nh.hasParam("param3_new"));

  double param1;
  double param2;
  double param3;
  nh.getParam("param1_new", param1);
  nh.getParam("param2_new", param2);
  nh.getParam("param3_new", param3);

  // Only old parameter is provided for param1.
  ASSERT_EQ(param1, 1.0);
  // Both old and new paramters are provided for param2. New one must be active.
  ASSERT_EQ(param2, 3.0);
  // Only new paramter is provided for param3.
  ASSERT_EQ(param3, 4.0);
}

TEST(Mcl3DlCompat, CompatMode)
{
  mcl_3dl_compat::supported_level = 2;
  mcl_3dl_compat::current_level = 3;
  mcl_3dl_compat::default_level = mcl_3dl_compat::supported_level;

  ros::NodeHandle("~").setParam("compatible", 2);
  ASSERT_NO_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("~").setParam("compatible", 3);
  ASSERT_NO_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("~").setParam("compatible", 4);
  ASSERT_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      },  // NOLINT(whitespace/braces)
      std::runtime_error);

  ros::NodeHandle("~").setParam("compatible", 1);
  ASSERT_THROW(
      {
        mcl_3dl_compat::checkCompatMode();
      },  // NOLINT(whitespace/braces)
      std::runtime_error);
}

class Mcl3DlCompatSubscribe
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std_msgs::Bool::ConstPtr msg_;
  mutable std_msgs::Bool::ConstPtr msg_const_;

  void cb(const std_msgs::Bool::ConstPtr& msg)
  {
    msg_ = msg;
  }
  void cbConst(const std_msgs::Bool::ConstPtr& msg) const
  {
    msg_const_ = msg;
  }

  Mcl3DlCompatSubscribe()
    : pnh_("~")
  {
  }
};

TEST(Mcl3DlCompat, Subscribe)
{
  mcl_3dl_compat::supported_level = 2;
  mcl_3dl_compat::current_level = 3;
  mcl_3dl_compat::default_level = mcl_3dl_compat::supported_level;

  Mcl3DlCompatSubscribe cls;

  ros::Publisher pub_old = cls.pnh_.advertise<std_msgs::Bool>("test_old", 1, true);
  ros::Publisher pub_new = cls.nh_.advertise<std_msgs::Bool>("test_new", 1, true);
  std_msgs::Bool msg;
  msg.data = false;
  pub_old.publish(msg);
  msg.data = true;
  pub_new.publish(msg);

  {
    cls.pnh_.setParam("compatible", 2);
    cls.msg_ = nullptr;
    ros::Subscriber sub = mcl_3dl_compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &Mcl3DlCompatSubscribe::cb, &cls);
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_));
    ASSERT_EQ(false, static_cast<bool>(cls.msg_->data));
  }

  {
    cls.pnh_.setParam("compatible", 3);
    cls.msg_ = nullptr;
    ros::Subscriber sub = mcl_3dl_compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &Mcl3DlCompatSubscribe::cb, &cls);
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_));
    ASSERT_EQ(true, static_cast<bool>(cls.msg_->data));
  }

  {
    cls.pnh_.setParam("compatible", 2);
    cls.msg_ = nullptr;
    ros::Subscriber sub = mcl_3dl_compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &Mcl3DlCompatSubscribe::cbConst, &cls);
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_const_));
    ASSERT_EQ(false, static_cast<bool>(cls.msg_const_->data));
  }

  {
    cls.pnh_.setParam("compatible", 3);
    cls.msg_ = nullptr;
    ros::Subscriber sub = mcl_3dl_compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &Mcl3DlCompatSubscribe::cbConst, &cls);
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_const_));
    ASSERT_EQ(true, static_cast<bool>(cls.msg_const_->data));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mcl_3dl_compat");

  return RUN_ALL_TESTS();
}

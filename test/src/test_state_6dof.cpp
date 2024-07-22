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

#include <cstddef>
#include <cmath>

#include <gtest/gtest.h>

#include <mcl_3dl/state_6dof.h>

TEST(State6DOF, Constructors)
{
  const mcl_3dl::State6DOF a(
      mcl_3dl::Vec3(1.0, 2.0, 3.0), mcl_3dl::Quat(0.001, 0.002, 0.003, 0.99));

  ASSERT_FALSE(a.isDiff());
  ASSERT_EQ(a.pos_, mcl_3dl::Vec3(1.0, 2.0, 3.0));
  ASSERT_EQ(a.rot_, mcl_3dl::Quat(0.001, 0.002, 0.003, 0.99));
  ASSERT_EQ(a.rot_, mcl_3dl::Quat(0.001, 0.002, 0.003, 0.99));
  ASSERT_EQ(a.odom_err_integ_lin_, mcl_3dl::Vec3(0.0, 0.0, 0.0));
  ASSERT_EQ(a.odom_err_integ_ang_, mcl_3dl::Vec3(0.0, 0.0, 0.0));

  const mcl_3dl::State6DOF b(
      mcl_3dl::Vec3(4.0, 5.0, 6.0), mcl_3dl::Vec3(0.1, 0.2, 0.3));

  ASSERT_TRUE(b.isDiff());
  ASSERT_EQ(b.pos_, mcl_3dl::Vec3(4.0, 5.0, 6.0));
  ASSERT_EQ(b.rot_, mcl_3dl::Quat(0.0, 0.0, 0.0, 1.0));
  ASSERT_EQ(b.rpy_.v_, mcl_3dl::Vec3(0.1, 0.2, 0.3));
  ASSERT_EQ(b.odom_err_integ_lin_, mcl_3dl::Vec3(0.0, 0.0, 0.0));
  ASSERT_EQ(b.odom_err_integ_ang_, mcl_3dl::Vec3(0.0, 0.0, 0.0));
}

TEST(State6DOF, Accessors)
{
  const mcl_3dl::State6DOF a(
      mcl_3dl::Vec3(1.0, 2.0, 3.0), mcl_3dl::Quat(0.001, 0.002, 0.003, 0.99));

  ASSERT_EQ(a[0], 1.0f);
  ASSERT_EQ(a[1], 2.0f);
  ASSERT_EQ(a[2], 3.0f);
  ASSERT_EQ(a[3], 0.001f);
  ASSERT_EQ(a[4], 0.002f);
  ASSERT_EQ(a[5], 0.003f);
  ASSERT_EQ(a[6], 0.99f);
  ASSERT_EQ(a[7], 0.0f);
  ASSERT_EQ(a[8], 0.0f);
  ASSERT_EQ(a[9], 0.0f);
  ASSERT_EQ(a[10], 0.0f);
  ASSERT_EQ(a[11], 0.0f);
  ASSERT_EQ(a[12], 0.0f);

  mcl_3dl::State6DOF b;
  for (size_t i = 0; i < b.size(); ++i)
    b[i] = 0.1 * (i + 1);
  for (size_t i = 0; i < b.size(); ++i)
    ASSERT_FLOAT_EQ(b[i], 0.1 * (i + 1));

  const mcl_3dl::State6DOF c = b;
  for (size_t i = 0; i < b.size(); ++i)
    ASSERT_FLOAT_EQ(c[i], 0.1 * (i + 1));
}

TEST(State6DOF, Adder)
{
  const mcl_3dl::State6DOF a(
      mcl_3dl::Vec3(1.0, 2.0, 3.0), mcl_3dl::Quat(mcl_3dl::Vec3(0.0, 0.0, 1.0), 0.1));
  const mcl_3dl::State6DOF b(
      mcl_3dl::Vec3(4.0, 5.0, 6.0), mcl_3dl::Quat(mcl_3dl::Vec3(0.0, 0.0, 1.0), 0.2));

  const mcl_3dl::State6DOF sum = a + b;
  const mcl_3dl::State6DOF sub = a - b;
  const mcl_3dl::Quat sum_rot(mcl_3dl::Vec3(0.0, 0.0, 1.0), 0.3);
  const mcl_3dl::Quat sub_rot(mcl_3dl::Vec3(0.0, 0.0, 1.0), -0.1);

  ASSERT_EQ(sum.pos_, mcl_3dl::Vec3(5.0, 7.0, 9.0));
  ASSERT_FLOAT_EQ(sum.rot_.x_, sum_rot.x_);
  ASSERT_FLOAT_EQ(sum.rot_.y_, sum_rot.y_);
  ASSERT_FLOAT_EQ(sum.rot_.z_, sum_rot.z_);
  ASSERT_FLOAT_EQ(sum.rot_.w_, sum_rot.w_);
  ASSERT_EQ(sub.pos_, mcl_3dl::Vec3(-3.0, -3.0, -3.0));
  ASSERT_FLOAT_EQ(sub.rot_.x_, sub_rot.x_);
  ASSERT_FLOAT_EQ(sub.rot_.y_, sub_rot.y_);
  ASSERT_FLOAT_EQ(sub.rot_.z_, sub_rot.z_);
  ASSERT_FLOAT_EQ(sub.rot_.w_, sub_rot.w_);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

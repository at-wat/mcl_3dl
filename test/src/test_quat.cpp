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

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstddef>

#include <mcl_3dl/quat.h>

#include <gtest/gtest.h>

TEST(Quat, Constractors)
{
  // Test quaternion elements style constructor and copy constructor
  const mcl_3dl::Quat a(1.0, 2.0, 3.0, 4.0);
  const mcl_3dl::Quat b(a);

  ASSERT_TRUE(a.x_ == 1.0);
  ASSERT_TRUE(a.y_ == 2.0);
  ASSERT_TRUE(a.z_ == 3.0);
  ASSERT_TRUE(a.w_ == 4.0);
  ASSERT_TRUE(b.x_ == 1.0);
  ASSERT_TRUE(b.y_ == 2.0);
  ASSERT_TRUE(b.z_ == 3.0);
  ASSERT_TRUE(b.w_ == 4.0);

  // Test RPY and angle-axis style constructor
  for (int i = 0; i < 3; ++i)
  {
    const float ang = M_PI / 2.0;
    const float r = (i == 0) ? 1.0 : 0.0;
    const float p = (i == 1) ? 1.0 : 0.0;
    const float y = (i == 2) ? 1.0 : 0.0;

    // Check consistency between styles
    const mcl_3dl::Quat c(mcl_3dl::Vec3(r * ang, p * ang, y * ang));
    mcl_3dl::Quat d;
    d.setRPY(mcl_3dl::Vec3(r * ang, p * ang, y * ang));
    ASSERT_TRUE(c == d);

    const mcl_3dl::Quat e(mcl_3dl::Vec3(r, p, y), ang);
    ASSERT_LT(fabs(c.x_ - e.x_), 1e-6);
    ASSERT_LT(fabs(c.y_ - e.y_), 1e-6);
    ASSERT_LT(fabs(c.z_ - e.z_), 1e-6);
    ASSERT_LT(fabs(c.w_ - e.w_), 1e-6);

    // Check elements
    ASSERT_LT(fabs(c.x_ - r * std::sqrt(2.0) / 2.0), 1e-6);
    ASSERT_LT(fabs(c.y_ - p * std::sqrt(2.0) / 2.0), 1e-6);
    ASSERT_LT(fabs(c.z_ - y * std::sqrt(2.0) / 2.0), 1e-6);
    ASSERT_LT(fabs(c.w_ - std::sqrt(2.0) / 2.0), 1e-6);

    // Check reverse conversions
    mcl_3dl::Vec3 crpy(c.getRPY());
    ASSERT_LT(fabs(r * ang - crpy.x_), 1e-3);
    ASSERT_LT(fabs(p * ang - crpy.y_), 1e-3);
    ASSERT_LT(fabs(y * ang - crpy.z_), 1e-3);

    mcl_3dl::Vec3 axis;
    float angle;
    c.getAxisAng(axis, angle);
    ASSERT_LT(fabs(r - axis.x_), 1e-6);
    ASSERT_LT(fabs(p - axis.y_), 1e-6);
    ASSERT_LT(fabs(y - axis.z_), 1e-6);
    ASSERT_LT(fabs(ang - angle), 1e-6);
  }

  // Test forward up style constructor
  for (int i = 0; i < 3; ++i)
  {
    const mcl_3dl::Vec3 fw((i == 0) ? 1.0 : 0.0, (i == 1) ? 1.0 : 0.0, (i == 2) ? 1.0 : 0.0);
    for (int j = 0; j < 3; ++j)
    {
      const mcl_3dl::Vec3 up((j == 0) ? 1.0 : 0.0, (j == 1) ? 1.0 : 0.0, (j == 2) ? 1.0 : 0.0);

      if (fw == up)
        continue;

      const mcl_3dl::Quat q(fw, up);

      // Rotate forward and up vectors and validate
      const mcl_3dl::Vec3 f = q * mcl_3dl::Vec3(1.0, 0.0, 0.0);
      const mcl_3dl::Vec3 u = q * mcl_3dl::Vec3(0.0, 0.0, 1.0);

      ASSERT_LT(fabs(f.x_ - fw.x_), 1e-6);
      ASSERT_LT(fabs(f.y_ - fw.y_), 1e-6);
      ASSERT_LT(fabs(f.z_ - fw.z_), 1e-6);
      ASSERT_LT(fabs(u.x_ - up.x_), 1e-6);
      ASSERT_LT(fabs(u.y_ - up.y_), 1e-6);
      ASSERT_LT(fabs(u.z_ - up.z_), 1e-6);
    }
  }
}

TEST(Quat, Operators)
{
  const mcl_3dl::Quat a(1.0, 2.0, 3.0, 4.0);

  // Test ==,!= operators
  ASSERT_TRUE(mcl_3dl::Quat(1.0, 2.0, 3.0, 4.0) == a);
  ASSERT_FALSE(mcl_3dl::Quat(1.0, 2.0, 3.0, 4.0) != a);

  for (uint32_t i = 1; i < (1 << 4); ++i)
  {
    const float xp = (i & (1 << 0)) ? 0.1 : 0.0;
    const float yp = (i & (1 << 1)) ? 0.1 : 0.0;
    const float zp = (i & (1 << 2)) ? 0.1 : 0.0;
    const float wp = (i & (1 << 3)) ? 0.1 : 0.0;
    ASSERT_TRUE(mcl_3dl::Quat(1.0 + xp, 2.0 + yp, 3.0 + zp, 4.0 + wp) != a);
    ASSERT_FALSE(mcl_3dl::Quat(1.0 + xp, 2.0 + yp, 3.0 + zp, 4.0 + wp) == a);
  }

  // Test +, -, +=, -= operators
  const mcl_3dl::Quat adding(0.5, -0.5, 1.0, -1.0);
  mcl_3dl::Quat a_plus = a;
  mcl_3dl::Quat a_minus = a;
  a_plus += adding;
  a_minus -= adding;
  ASSERT_TRUE(a + adding == mcl_3dl::Quat(1.5, 1.5, 4.0, 3.0));
  ASSERT_TRUE(a + adding == a_plus);
  ASSERT_TRUE(a - adding == mcl_3dl::Quat(0.5, 2.5, 2.0, 5.0));
  ASSERT_TRUE(a - adding == a_minus);

  // Test -() operator
  ASSERT_TRUE(-a == mcl_3dl::Quat(-1.0, -2.0, -3.0, -4.0));

  // Test scalar *, / operators
  mcl_3dl::Quat a_mul = a;
  mcl_3dl::Quat a_div = a;
  a_mul *= 0.5;
  a_div /= 2.0;
  ASSERT_TRUE(a * 0.5 == mcl_3dl::Quat(0.5, 1.0, 1.5, 2.0));
  ASSERT_TRUE(a / 2.0 == a * 0.5);
  ASSERT_TRUE(a * 0.5 == a_mul);
  ASSERT_TRUE(a / 2.0 == a_div);
}

TEST(Quat, Norm)
{
  // Check norm operations
  const mcl_3dl::Quat a(1.0, 2.0, 3.0, 4.0);
  const mcl_3dl::Quat b(-4.0, 5.0, 6.0, -7.0);
  ASSERT_LT(fabs(a.norm() - 5.477226), 1e-6);
  ASSERT_LT(fabs(b.norm() - 11.224972), 1e-6);
  ASSERT_LT(fabs(a.normalized().norm() - 1.0), 1e-6);
  ASSERT_LT(fabs(b.normalized().norm() - 1.0), 1e-6);
}

TEST(Quat, Products)
{
  // Check cross and dot products
  const int num_samples = 8;
  const mcl_3dl::Quat samples[num_samples] =
      {
          mcl_3dl::Quat(mcl_3dl::Vec3(1.5, 2.5, 3.5), 0.5),
          mcl_3dl::Quat(mcl_3dl::Vec3(-0.5, 1.0, 1.0), 1.0),
          mcl_3dl::Quat(mcl_3dl::Vec3(0.5, -1.0, 2.0), 1.5),
          mcl_3dl::Quat(mcl_3dl::Vec3(0.5, 1.0, -2.0), 2.0),
          mcl_3dl::Quat(mcl_3dl::Vec3(-2.0, -5.0, 4.0), 2.5),
          mcl_3dl::Quat(mcl_3dl::Vec3(2.0, -5.0, -4.0), -1.0),
          mcl_3dl::Quat(mcl_3dl::Vec3(-2.0, 5.0, -4.0), -1.5),
          mcl_3dl::Quat(mcl_3dl::Vec3(-3.0, -1.0, -2.0), -2.0),
      };

  // Check inverse
  for (int i = 0; i < num_samples; ++i)
  {
    const mcl_3dl::Quat inv = samples[i].inv();
    const mcl_3dl::Quat ident0 = inv * samples[i];
    const mcl_3dl::Quat ident1 = samples[i] * inv;

    ASSERT_LT(fabs(ident0.x_), 1e-6);
    ASSERT_LT(fabs(ident0.y_), 1e-6);
    ASSERT_LT(fabs(ident0.z_), 1e-6);
    ASSERT_LT(fabs(ident0.w_ - 1.0), 1e-6);
    ASSERT_LT(fabs(ident1.x_), 1e-6);
    ASSERT_LT(fabs(ident1.y_), 1e-6);
    ASSERT_LT(fabs(ident1.z_), 1e-6);
    ASSERT_LT(fabs(ident1.w_ - 1.0), 1e-6);
  }

  // Check rotate axis
  for (int i = 0; i < num_samples; ++i)
  {
    for (int j = 0; j < num_samples; ++j)
    {
      mcl_3dl::Vec3 a_axis;
      float a_ang;
      samples[i].getAxisAng(a_axis, a_ang);

      mcl_3dl::Quat a(a_axis, a_ang);
      const mcl_3dl::Quat& b = samples[j];

      const mcl_3dl::Vec3 rotated_axis = b * a_axis;
      a.rotateAxis(b);

      mcl_3dl::Vec3 a_axis2;
      float a_ang2;
      a.getAxisAng(a_axis2, a_ang2);

      ASSERT_LT(fabs(a_axis2.x_ - rotated_axis.x_), 1e-6);
      ASSERT_LT(fabs(a_axis2.y_ - rotated_axis.y_), 1e-6);
      ASSERT_LT(fabs(a_axis2.z_ - rotated_axis.z_), 1e-6);
      ASSERT_LT(fabs(a_ang - a_ang2), 1e-6);
    }
  }

  // Check vector rotation
  const int num_vecs = 3;
  const mcl_3dl::Vec3 v[num_vecs] =
      {
          mcl_3dl::Vec3(1.0, 0.0, 0.0),
          mcl_3dl::Vec3(0.0, 1.0, 0.0),
          mcl_3dl::Vec3(0.0, 0.0, 1.0),
      };
  const int num_rots = 3;
  const mcl_3dl::Quat r[num_rots] =
      {
          mcl_3dl::Quat(mcl_3dl::Vec3(1.0, 0.0, 0.0), M_PI / 2.0),
          mcl_3dl::Quat(mcl_3dl::Vec3(0.0, 1.0, 0.0), -M_PI / 2.0),
          mcl_3dl::Quat(mcl_3dl::Vec3(0.0, 0.0, 1.0), -M_PI / 2.0),
      };
  const mcl_3dl::Vec3 v_ans[3][3] =
      {
          mcl_3dl::Vec3(1.0, 0.0, 0.0),
          mcl_3dl::Vec3(0.0, 0.0, 1.0),
          mcl_3dl::Vec3(0.0, -1.0, 0.0),
          mcl_3dl::Vec3(0.0, 0.0, 1.0),
          mcl_3dl::Vec3(0.0, 1.0, 0.0),
          mcl_3dl::Vec3(-1.0, 0.0, 0.0),
          mcl_3dl::Vec3(0.0, -1.0, 0.0),
          mcl_3dl::Vec3(1.0, 0.0, 0.0),
          mcl_3dl::Vec3(0.0, 0.0, 1.0),
      };
  for (int i = 0; i < num_vecs; ++i)
  {
    for (int j = 0; j < num_rots; ++j)
    {
      const mcl_3dl::Vec3 result = r[j] * v[i];
      ASSERT_LT(fabs(result.x_ - v_ans[j][i].x_), 1e-6);
      ASSERT_LT(fabs(result.y_ - v_ans[j][i].y_), 1e-6);
      ASSERT_LT(fabs(result.z_ - v_ans[j][i].z_), 1e-6);
    }
  }

  for (int i = 0; i < num_samples; ++i)
  {
    for (int j = 0; j < num_samples; ++j)
    {
      const mcl_3dl::Quat& a = samples[i];
      const mcl_3dl::Quat& b = samples[j];

      // Check dot products based on the distributive property
      ASSERT_LT((a - b).dot(a - b) - a.dot(a) - b.dot(b) + 2.0 * a.dot(b), 1e-6);

      // Check mcl_3dl::Quat by mcl_3dl::Quat products
      const mcl_3dl::Vec3 v(1.0, 2.0, 3.0);
      const mcl_3dl::Vec3 v1 = (a * b) * v;
      const mcl_3dl::Vec3 v2 = a * (b * v);
      ASSERT_LT(fabs(v1.x_ - v2.x_), 1e-6);
      ASSERT_LT(fabs(v1.y_ - v2.y_), 1e-6);
      ASSERT_LT(fabs(v1.z_ - v2.z_), 1e-6);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

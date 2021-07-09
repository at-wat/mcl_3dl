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

#include <cstddef>
#include <cmath>

#include <gtest/gtest.h>

#include <mcl_3dl/vec3.h>

TEST(Vec3, Constructors)
{
  // Test vector elements constructor and copy constructor
  const mcl_3dl::Vec3 a(1.0, 2.0, 3.0);
  const mcl_3dl::Vec3 b(a);

  // Test elements
  ASSERT_TRUE(a.x_ == 1.0);
  ASSERT_TRUE(a.y_ == 2.0);
  ASSERT_TRUE(a.z_ == 3.0);
  ASSERT_TRUE(b.x_ == 1.0);
  ASSERT_TRUE(b.y_ == 2.0);
  ASSERT_TRUE(b.z_ == 3.0);
}

TEST(Vec3, Operators)
{
  const mcl_3dl::Vec3 a(1.0, 2.0, 3.0);

  // Test ==,!= operators
  ASSERT_TRUE(mcl_3dl::Vec3(1.0, 2.0, 3.0) == a);
  ASSERT_FALSE(mcl_3dl::Vec3(1.0, 2.0, 3.0) != a);

  for (uint32_t i = 1; i < (1 << 3); i++)
  {
    const float xp = (i & (1 << 0)) ? 0.1 : 0.0;
    const float yp = (i & (1 << 1)) ? 0.1 : 0.0;
    const float zp = (i & (1 << 2)) ? 0.1 : 0.0;
    ASSERT_TRUE(mcl_3dl::Vec3(1.0 + xp, 2.0 + yp, 3.0 + zp) != a);
    ASSERT_FALSE(mcl_3dl::Vec3(1.0 + xp, 2.0 + yp, 3.0 + zp) == a);
  }

  // Test +, -, +=, -= operators
  const mcl_3dl::Vec3 adding(0.5, -0.5, 1.0);
  mcl_3dl::Vec3 a_plus = a;
  mcl_3dl::Vec3 a_minus = a;
  a_plus += adding;
  a_minus -= adding;
  ASSERT_TRUE(a + adding == mcl_3dl::Vec3(1.5, 1.5, 4.0));
  ASSERT_TRUE(a + adding == a_plus);
  ASSERT_TRUE(a - adding == mcl_3dl::Vec3(0.5, 2.5, 2.0));
  ASSERT_TRUE(a - adding == a_minus);

  // Test -() operator
  ASSERT_TRUE(-a == mcl_3dl::Vec3(-1.0, -2.0, -3.0));

  // Test scalar *, / operators
  mcl_3dl::Vec3 a_mul = a;
  mcl_3dl::Vec3 a_div = a;
  a_mul *= 0.5;
  a_div /= 2.0;
  ASSERT_TRUE(a * 0.5 == mcl_3dl::Vec3(0.5, 1.0, 1.5));
  ASSERT_TRUE(a / 2.0 == a * 0.5);
  ASSERT_TRUE(a * 0.5 == a_mul);
  ASSERT_TRUE(a / 2.0 == a_div);

  // Test [] operator
  ASSERT_EQ(1.0, a[0]);
  ASSERT_EQ(2.0, a[1]);
  ASSERT_EQ(3.0, a[2]);
  ASSERT_EQ(1.0, a[3]);  // exceeded index points 0
}

TEST(Vec3, Times)
{
  // Check times operation (element-by-element multiplication)
  const mcl_3dl::Vec3 a(1.0, 2.0, 3.0);
  const mcl_3dl::Vec3 b(-4.0, 5.0, 6.0);
  ASSERT_TRUE(a.times(b) == mcl_3dl::Vec3(-4.0, 10.0, 18.0));
}

TEST(Vec3, Norm)
{
  // Check norm operations
  const mcl_3dl::Vec3 a(1.0, 2.0, 3.0);
  const mcl_3dl::Vec3 b(-4.0, 5.0, 6.0);
  ASSERT_LT(fabs(a.norm() - 3.741657), 1e-6);
  ASSERT_LT(fabs(b.norm() - 8.774964), 1e-6);
  ASSERT_LT(fabs(a.normalized().norm() - 1.0), 1e-6);
  ASSERT_LT(fabs(b.normalized().norm() - 1.0), 1e-6);
}

TEST(Vec3, Products)
{
  // Check cross and dot products
  const int num_samples = 8;
  const mcl_3dl::Vec3 samples[num_samples] =
      {
          mcl_3dl::Vec3(1.5, 2.5, 3.5),
          mcl_3dl::Vec3(-0.5, 1.0, 1.0),
          mcl_3dl::Vec3(0.5, -1.0, 2.0),
          mcl_3dl::Vec3(0.5, 1.0, -2.0),
          mcl_3dl::Vec3(-2.0, -5.0, 4.0),
          mcl_3dl::Vec3(2.0, -5.0, -4.0),
          mcl_3dl::Vec3(-2.0, 5.0, -4.0),
          mcl_3dl::Vec3(-3.0, -1.0, -2.0),
      };

  for (int i = 0; i < num_samples; ++i)
  {
    for (int j = 0; j < num_samples; ++j)
    {
      const mcl_3dl::Vec3& a = samples[i];
      const mcl_3dl::Vec3& b = samples[j];

      // Check dot products based on the distributive property
      ASSERT_LT((a - b).dot(a - b) - a.dot(a) - b.dot(b) + 2.0 * a.dot(b), 1e-6);

      // Check cross products based on the geometric meaning
      ASSERT_LT(a.dot(a.cross(b)), 1e-6);
      ASSERT_LT(a.dot(a.cross(b)), 1e-6);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

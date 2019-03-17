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

#include <gtest/gtest.h>

#include <mcl_3dl/filter.h>

TEST(Filter, LPFCharacteristic)
{
  for (int time_const = 20; time_const < 100; time_const += 20)
  {
    mcl_3dl::Filter lpf(mcl_3dl::Filter::FILTER_LPF, time_const, 0.0);
    ASSERT_LT(fabs(lpf.get()), 1e-6);

    // Input step function
    float ret = 0;
    for (int i = 0; i < time_const; ++i)
    {
      ret = lpf.in(1.0);
    }
    // Check value at 1 time unit
    ASSERT_TRUE(ret == lpf.get());
    ASSERT_LT(fabs(ret - (1.0 - expf(-1.0))), 1e-2);

    for (int i = time_const; i < time_const * 100; ++i)
    {
      ret = lpf.in(1.0);
    }
    // Check value at inf time
    ASSERT_TRUE(ret == lpf.get());
    ASSERT_LT(fabs(ret - 1.0), 1e-2);

    // Check set
    lpf.set(1.0);
    ASSERT_LT(fabs(lpf.get() - 1.0), 1e-2);
    lpf.in(1.0);
    ASSERT_LT(fabs(lpf.get() - 1.0), 1e-2);
  }
}

TEST(Filter, HPFCharacteristic)
{
  for (int time_const = 20; time_const < 100; time_const += 20)
  {
    mcl_3dl::Filter lpf(mcl_3dl::Filter::FILTER_LPF, time_const, 0.0);
    mcl_3dl::Filter hpf(mcl_3dl::Filter::FILTER_HPF, time_const, 0.0);

    // Input step function
    for (int i = 0; i < time_const * 10; ++i)
    {
      float ret_h, ret_l;
      ret_l = lpf.in(1.0);
      ret_h = hpf.in(1.0);

      // Check complementarity against LPF
      ASSERT_LT(fabs(ret_l + ret_h - 1.0), 1e-2);
    }
  }
}

TEST(Filter, AugleLPF)
{
  for (float zero = 0.0; zero < M_PI * 2 * 4; zero += M_PI * 2)
  {
    // Check 0.5 rad to 2pi - 0.5 rad transition
    const float start1 = zero + 0.5;
    const float end1 = zero + M_PI * 2.0 - 0.5;

    mcl_3dl::Filter lpf(mcl_3dl::Filter::FILTER_LPF, 10, start1);
    mcl_3dl::Filter lpf_angle(mcl_3dl::Filter::FILTER_LPF, 10, start1, true);
    ASSERT_LT(fabs(lpf.get() - start1), 1e-6);
    ASSERT_LT(fabs(lpf_angle.get() - start1), 1e-6);

    for (int i = 0; i < 100; ++i)
    {
      lpf.in(end1);
      lpf_angle.in(end1);
      ASSERT_GT(lpf.get(), start1);
      ASSERT_LT(lpf_angle.get(), start1);
    }
    ASSERT_LT(fabs(lpf.get() - end1), 1e-2);
    ASSERT_LT(fabs(lpf_angle.get() - (zero - 0.5)), 1e-2);

    // Check 2pi - 0.5 to 0.5 rad transition
    const float start2 = zero - 0.5;
    const float end2 = zero - M_PI * 2.0 + 0.5;

    lpf.set(start2);
    lpf_angle.set(start2);
    ASSERT_LT(fabs(lpf.get() - start2), 1e-6);
    ASSERT_LT(fabs(lpf_angle.get() - start2), 1e-6);

    for (int i = 0; i < 100; ++i)
    {
      lpf.in(end2);
      lpf_angle.in(end2);
      ASSERT_LT(lpf.get(), start2);
      ASSERT_GT(lpf_angle.get(), start2);
    }
    ASSERT_LT(fabs(lpf.get() - end2), 1e-2);
    ASSERT_LT(fabs(lpf_angle.get() - (zero + 0.5)), 1e-2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

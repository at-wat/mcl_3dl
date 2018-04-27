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

#include <mcl_3dl/nd.h>

TEST(NormalLiklihoodTest, testNormality)
{
  for (float sigma = 1.0; sigma <= 3.0; sigma += 1.0)
  {
    // Check distribution
    mcl_3dl::NormalLikelihood<float> nl(sigma);
    const float likelihood0 = 1.0 / sqrtf(M_PI * 2.0 * sigma * sigma);
    ASSERT_LT(fabs(nl(0.0) - likelihood0), 1e-6);
    ASSERT_LT(fabs(nl(sigma) - likelihood0 * 0.60653066), 1e-6);
    ASSERT_LT(fabs(nl(-sigma) - likelihood0 * 0.60653066), 1e-6);
    ASSERT_LT(fabs(nl(3.0 * sigma) - likelihood0 * 0.011108997), 1e-6);
    ASSERT_LT(fabs(nl(-3.0 * sigma) - likelihood0 * 0.011108997), 1e-6);

    // Check integrated value
    float sum(0.0);
    const float step = 0.01;
    for (float i = -100.0; i < 100.0; i += step)
    {
      sum += nl(i) * step;
    }
    ASSERT_LT(fabs(sum) - 1.0, 1e-6);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

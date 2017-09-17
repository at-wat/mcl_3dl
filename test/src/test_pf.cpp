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
#include <vector>

#include <gtest/gtest.h>

#include <pf.h>

TEST(PfTest, testVariableParticleSize)
{
  class State : public pf::ParticleBase<float>
  {
  public:
    float x;
    float &operator[](const size_t i)override
    {
      switch (i)
      {
        case 0:
          return x;
      }
      return x;
    }
    const float &operator[](const size_t i) const
    {
      switch (i)
      {
        case 0:
          return x;
      }
      return x;
    }
    size_t size() const override
    {
      return 1;
    };
    explicit State(const float x)
    {
      this->x = x;
    }
    State()
    {
      x = 0;
    }
    void normalize()
    {
    }
  };

  const size_t size_num = 3;
  const size_t size[size_num] =
      {
        1024, 2048, 900
      };
  pf::ParticleFilter<State, float> pf(size[0]);

  const float center = 12.3;
  const float sigma = 0.45;
  pf.init(
      State(center),
      State(sigma));

  for (size_t i = 0; i < size_num; ++i)
  {
    ASSERT_EQ(pf.getParticleSize(), size[i]);

    const State e = pf.expectation();
    const std::vector<State> v = pf.covariance();
    ASSERT_LT(fabs(e[0] - center), 1e-1);
    ASSERT_LT(fabs(sqrtf(v[0][0]) - sigma), 1e-1);

    pf.resample(State());

    const State e_r = pf.expectation();
    const std::vector<State> v_r = pf.covariance();
    ASSERT_LT(fabs(e_r[0] - center), 1e-1);
    ASSERT_LT(fabs(sqrtf(v_r[0][0]) - sigma), 1e-1);

    if (i + 1 != size_num)
    {
      pf.resizeParticle(size[i + 1]);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

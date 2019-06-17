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

#include <mcl_3dl/pf.h>
#include <mcl_3dl/nd.h>

class State : public mcl_3dl::pf::ParticleBase<float>
{
public:
  float x;
  float& operator[](const size_t i)override
  {
    switch (i)
    {
      case 0:
        return x;
    }
    return x;
  }
  const float& operator[](const size_t i) const
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

TEST(Pf, BayesianEstimation)
{
  mcl_3dl::pf::ParticleFilter<State, float> pf(1024);
  const float center_list[] =
      {
        10.0, 11.0, 9.5
      };

  const float abs_error = 2e-1;
  const float sigma = 1.0;
  const float sigma2 = 2.0;

  for (auto center : center_list)
  {
    for (auto center2 : center_list)
    {
      pf.init(
          State(center),
          State(sigma));

      ASSERT_NEAR(center, pf.expectation()[0], abs_error);
      ASSERT_NEAR(sigma, pf.covariance()[0][0], abs_error);

      auto likelihood = [center2, sigma2](const State& s) -> float
      {
        return exp(-powf(s[0] - center2, 2.0) / (2.0 * powf(sigma2, 2.0)));
      };
      pf.measure(likelihood);

      // Numerical representation
      const int HISTOGRAM_SIZE = 4000;
      const float HISTOGRAM_RESOLUTION = 0.02;

      float dist[HISTOGRAM_SIZE];
      mcl_3dl::NormalLikelihood<float> nd1(sigma);
      mcl_3dl::NormalLikelihood<float> nd2(sigma2);
      double avg = 0;
      float total = 0;
      for (int i = 0; i < HISTOGRAM_SIZE; i++)
      {
        const float x = (i - HISTOGRAM_SIZE / 2.0) * HISTOGRAM_RESOLUTION;
        dist[i] = nd1(x - center) * nd2(x - center2);

        avg += dist[i] * x;
        total += dist[i];
      }
      avg /= total;
      double var = 0;
      for (int i = 0; i < HISTOGRAM_SIZE; i++)
      {
        const float x = (i - HISTOGRAM_SIZE / 2.0) * HISTOGRAM_RESOLUTION - avg;
        var += powf(x, 2.0) * dist[i];
      }
      var /= total;

      // Compare with numerical result
      ASSERT_NEAR(avg, pf.expectation()[0], abs_error);
      ASSERT_NEAR(var, pf.covariance()[0][0], abs_error);

      // Try resampling
      pf.resample(State(0.0));
      ASSERT_NEAR(avg, pf.expectation()[0], abs_error);
      ASSERT_NEAR(var, pf.covariance()[0][0], abs_error);
    }
  }
}

TEST(Pf, VariableParticleSize)
{
  const size_t size_num = 3;
  const size_t size[size_num] =
      {
        1024, 2048, 900
      };
  mcl_3dl::pf::ParticleFilter<State, float> pf(size[0]);

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

TEST(Pf, ResampleFlatLikelihood)
{
  mcl_3dl::pf::ParticleFilter<State, float> pf(10);
  const float center = 12.3;
  const float sigma = 0.45;
  pf.init(
      State(center),
      State(sigma));

  std::vector<float> orig;

  for (size_t i = 0; i < pf.getParticleSize(); ++i)
    orig.push_back(pf.getParticle(i)[0]);

  pf.resample(State());

  for (size_t i = 0; i < pf.getParticleSize(); ++i)
    ASSERT_EQ(pf.getParticle(i)[0], orig[i]);
}

TEST(Pf, ResampleFirstParticle)
{
  const std::vector<float> probs =
  {
    0.0001f, 0.2f, 0.2f, 0.2f, 0.3999f
  };
  const std::vector<float> states =
  {
    0.0f, 1.0f, 2.0f, 3.0f, 4.0f
  };
  const std::vector<float> expected_resampled_states =
  {
    1.0f, 2.0f, 3.0f, 4.0f, 4.0f
  };
  const size_t particle_num = probs.size();

  mcl_3dl::pf::ParticleFilter<State, float> pf(particle_num);
  auto it = pf.begin();
  for (size_t i = 0; i < particle_num; ++i, ++it)
  {
    it->state_.x = states.at(i);
    it->probability_ = probs.at(i);
  }
  pf.resample(State());

  ASSERT_EQ(particle_num, pf.getParticleSize());
  for (size_t i = 0; i < pf.getParticleSize(); ++i)
  {
    EXPECT_FLOAT_EQ(expected_resampled_states.at(i), pf.getParticle(i)[0]);
  }
}

TEST(Pf, Iterators)
{
  mcl_3dl::pf::ParticleFilter<State, float> pf(10);
  const float val0 = 12.3;
  const float val1 = 45.6;
  pf.init(
      State(val0),
      State(0.0));

  for (auto it = pf.begin(); it != pf.end(); ++it)
  {
    ASSERT_EQ(it->state_[0], val0);
    it->state_[0] = val1;
  }
  for (auto it = pf.begin(); it != pf.end(); ++it)
    ASSERT_EQ(it->state_[0], val1);
}

TEST(Pf, AppendParticles)
{
  mcl_3dl::pf::ParticleFilter<State, float> pf(10);
  const float val0 = 12.3;
  const float val1 = 45.6;
  pf.init(
      State(val0),
      State(0.0));
  // particles 0-9 has val0

  for (auto it = pf.appendParticle(10); it != pf.end(); ++it)
    it->state_[0] = val1;
  // appended particles 10-19 has val1

  ASSERT_EQ(pf.getParticleSize(), 20u);
  for (size_t i = 0; i < 10; ++i)
    ASSERT_EQ(pf.getParticle(i)[0], val0);
  for (size_t i = 10; i < 20; ++i)
    ASSERT_EQ(pf.getParticle(i)[0], val1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

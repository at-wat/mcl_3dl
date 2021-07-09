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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <mcl_3dl/noise_generators/diagonal_noise_generator.h>
#include <mcl_3dl/noise_generators/multivariate_noise_generator.h>
#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
template <typename NOISE_GEN>
void testNoiseGeneratorResults(const std::vector<float>& expected_means,
                               const std::vector<float>& expected_covariances,
                               const NOISE_GEN& gen,
                               const float allowable_error_of_zero_cov)
{
  const size_t dim = expected_means.size();
  ASSERT_EQ(dim, gen.getDimension());
  ASSERT_EQ(dim * dim, expected_covariances.size());

  std::mt19937 mt(123);
  std::vector<std::vector<float>> results(dim);
  for (size_t i = 0; i < 10000; ++i)
  {
    const auto result = gen(mt);
    ASSERT_EQ(dim, result.size());
    for (size_t j = 0; j < dim; ++j)
    {
      results[j].push_back(result[j]);
    }
  }
  std::vector<float> averages;
  for (size_t i = 0; i < dim; ++i)
  {
    const float average = std::accumulate(results[i].begin(), results[i].end(), 0.0) / results[i].size();
    EXPECT_NEAR(average, expected_means[i], 0.1);
    averages.push_back(average);
  }
  for (size_t i = 0; i < dim; ++i)
  {
    for (size_t j = i; j < dim; ++j)
    {
      float covar = 0;
      for (size_t n = 0; n < results[i].size(); ++n)
      {
        covar += (results[i][n] - averages[i]) * (results[j][n] - averages[j]);
      }
      covar /= results[i].size();
      const float expected_covar = expected_covariances[i + j * dim];
      if (expected_covar == 0.0)
      {
        EXPECT_NEAR(covar, expected_covar, allowable_error_of_zero_cov) << "Error at " << i << "," << j;
      }
      else
      {
        EXPECT_GE(covar, expected_covariances[i + j * dim] * 0.9 * 0.9) << "Error at " << i << "," << j;
        EXPECT_LE(covar, expected_covariances[i + j * dim] * 1.1 * 1.1) << "Error at " << i << "," << j;
      }
    }
  }
}

std::vector<float> toCovarianceMatrix(const std::vector<float>& sigma)
{
  const size_t dim = sigma.size();

  std::vector<float> result(dim * dim, 0.0);
  for (size_t i = 0; i < sigma.size(); ++i)
  {
    result[i + i * dim] = sigma[i] * sigma[i];
  }
  return result;
}

TEST(NoiseGenerator, DiagonalNoiseGenerator)
{
  std::vector<float> expected_mean;
  expected_mean.push_back(7.0);
  expected_mean.push_back(8.0);
  expected_mean.push_back(9.0);

  std::vector<float> expected_sigma;
  expected_sigma.push_back(1.0);
  expected_sigma.push_back(2.0);
  expected_sigma.push_back(3.0);

  const DiagonalNoiseGenerator<float> gen(expected_mean, expected_sigma);
  testNoiseGeneratorResults(expected_mean, toCovarianceMatrix(expected_sigma), gen, 0.1);
}

TEST(NoiseGenerator, MultivariateNoiseGenerator)
{
  const std::vector<float> expected_mean = {-1.0, 2.0, -3.0};
  const std::vector<float> expected_covariance =
      {
          1.0, 0.3, 0.7,  //
          0.3, 2.0, 0.4,  //
          0.7, 0.4, 1.0,  //
      };

  const MultivariateNoiseGenerator<float> gen(expected_mean, expected_covariance);
  testNoiseGeneratorResults(expected_mean, expected_covariance, gen, 0.01);
}

TEST(NoiseGenerator, MultivariateNoiseGeneratorForState6Dof)
{
  const std::vector<float> expected_mean = {5.0, -6.0, 7.0, -0.3, 0.2, 0.1};
  const std::vector<float> expected_covariance =
      {
          2.000, 0.500, 0.600, 0.050, 0.040, 0.000,  //
          0.500, 2.500, 0.400, 0.060, 0.070, 0.080,  //
          0.600, 0.400, 3.000, 0.090, 0.020, 0.110,  //
          0.050, 0.060, 0.090, 0.200, 0.045, 0.035,  //
          0.040, 0.070, 0.020, 0.045, 0.150, 0.015,  //
          0.000, 0.080, 0.110, 0.035, 0.015, 0.100,  //
      };

  const mcl_3dl::Vec3 mean_pos(expected_mean[0], expected_mean[1], expected_mean[2]);
  const mcl_3dl::Quat mean_rot(mcl_3dl::Vec3(expected_mean[3], expected_mean[4], expected_mean[5]));
  const State6DOF mean(mean_pos, mean_rot);

  const MultivariateNoiseGenerator<float> gen(mean, expected_covariance);
  testNoiseGeneratorResults(expected_mean, expected_covariance, gen, 0.01);
}

}  // namespace mcl_3dl

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

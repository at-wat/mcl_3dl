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

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <mcl_3dl/noise_generator.h>
#include <algorithm>

namespace mcl_3dl
{
TEST(NoiseGenerator, DiagonalNoiseGenerator)
{
  const size_t dim = 3;
  std::vector<double> expected_sigma;
  expected_sigma.push_back(1.0);
  expected_sigma.push_back(2.0);
  expected_sigma.push_back(3.0);

  DiagonalNoiseGenerator<double> gen(expected_sigma);
  ASSERT_EQ(dim, gen.getDimension());

  std::mt19937 mt(123);
  std::vector<std::vector<double>> results(dim);
  for (size_t i = 0; i < 1000; ++i)
  {
    const auto result = gen(mt);
    ASSERT_EQ(3, result.size());
    for (size_t j = 0; j < dim; ++j)
    {
      results[j].push_back(result[j]);
    }
  }
  std::vector<double> averages;
  for (size_t i = 0; i < dim; ++i)
  {
    const double average = std::accumulate(results[i].begin(), results[i].end(), 0.0) / results[i].size();
    EXPECT_GE(average, -0.1);
    EXPECT_LE(average, 0.1);
    averages.push_back(average);
  }
  for (size_t i = 0; i < dim; ++i)
  {
    const double average = averages[i];
    // clang-format off
    const auto calc_var = [average](double prev, double val)
    {
      return prev + std::pow((val - average), 2);
    };
    // clang-format on
    const double var = std::accumulate(results[i].begin(), results[i].end(), 0.0, calc_var) / results[i].size();
    const double sigma = std::sqrt(var);
    EXPECT_GE(sigma, expected_sigma[i] * 0.9);
    EXPECT_LE(sigma, expected_sigma[i] * 1.1);
  }
}

TEST(NoiseGenerator, MultivariateNoiseGenerator)
{
  const size_t dim = 3;
  // clang-format off
  const std::vector<double> expected_covariances =
  {
      1.0, 0.3, 0.7,
      0.3, 2.0, 0.4,
      0.7, 0.4, 1.0
  };
  // clang-format on

  const MultivariateNoiseGenerator<double> gen(expected_covariances);
  ASSERT_EQ(dim, gen.getDimension());

  std::mt19937 mt(123);
  std::vector<std::vector<double>> results(dim);
  for (size_t i = 0; i < 1000; ++i)
  {
    const auto result = gen(mt);
    ASSERT_EQ(3, result.size());
    for (size_t j = 0; j < dim; ++j)
    {
      results[j].push_back(result[j]);
    }
  }
  std::vector<double> averages;
  for (size_t i = 0; i < dim; ++i)
  {
    const double average = std::accumulate(results[i].begin(), results[i].end(), 0.0) / results[i].size();
    EXPECT_GE(average, -0.1);
    EXPECT_LE(average, 0.1);
    averages.push_back(average);
  }
  for (size_t i = 0; i < dim; ++i)
  {
    for (size_t j = i; j < dim; ++j)
    {
      double covar = 0;
      for (size_t n = 0; n < results[i].size(); ++n)
      {
        covar += (results[i][n] - averages[i]) * (results[j][n] - averages[j]);
      }
      covar /= results[i].size();
      EXPECT_GE(covar, expected_covariances[i + j * dim] * 0.9 * 0.9);
      EXPECT_LE(covar, expected_covariances[i + j * dim] * 1.1 * 1.1);
    }
  }
}
}  // namespace mcl_3dl

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

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

#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <mcl_3dl/nd.h>

TEST(NormalLiklihood, Normality)
{
  for (double sigma = 1.0; sigma <= 3.0; sigma += 1.0)
  {
    // Check distribution
    mcl_3dl::NormalLikelihood<double> nl(sigma);
    const double likelihood0 = 1.0 / std::sqrt(M_PI * 2.0 * sigma * sigma);
    ASSERT_NEAR(nl(0.0), likelihood0, 1e-6);
    ASSERT_NEAR(nl(sigma), likelihood0 * 0.60653066, 1e-6);
    ASSERT_NEAR(nl(-sigma), likelihood0 * 0.60653066, 1e-6);
    ASSERT_NEAR(nl(3.0 * sigma), likelihood0 * 0.011108997, 1e-6);
    ASSERT_NEAR(nl(-3.0 * sigma), likelihood0 * 0.011108997, 1e-6);

    // Check integrated value
    double sum(0.0);
    const double step = 0.1;
    for (double i = -100.0; i < 100.0; i += step)
    {
      sum += nl(i) * step;
    }
    ASSERT_NEAR(sum, 1.0, 1e-6);
  }
}

TEST(NormalLiklihood, NormalityNd)
{
  for (double sigma = 1.0; sigma <= 3.0; sigma += 1.0)
  {
    // Check distribution
    using NormalLikelihood2d = mcl_3dl::NormalLikelihoodNd<double, 2>;
    NormalLikelihood2d::Matrix cov;
    cov << sigma, 0,
        0, sigma * 2;
    NormalLikelihood2d nl(cov);

    const double likelihood0 = 1.0 / (M_PI * 2.0 * sqrt(cov.determinant()));
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(0.0, 0.0)), likelihood0, 1e-6);

    const double e1a =
        exp(-0.5 *
            NormalLikelihood2d::Vector(sigma, 0.0).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(sigma, 0.0));
    const double e1b =
        exp(-0.5 *
            NormalLikelihood2d::Vector(0.0, sigma).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(0.0, sigma));
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(sigma, 0.0)), likelihood0 * e1a, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(-sigma, 0.0)), likelihood0 * e1a, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(0.0, sigma)), likelihood0 * e1b, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(0.0, -sigma)), likelihood0 * e1b, 1e-6);

    const double e2a =
        exp(-0.5 *
            NormalLikelihood2d::Vector(3.0 * sigma, 0.0).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(3.0 * sigma, 0.0));
    const double e2b =
        exp(-0.5 *
            NormalLikelihood2d::Vector(0.0, 3.0 * sigma).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(0.0, 3.0 * sigma));
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(3.0 * sigma, 0.0)), likelihood0 * e2a, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(-3.0 * sigma, 0.0)), likelihood0 * e2a, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(0.0, 3.0 * sigma)), likelihood0 * e2b, 1e-6);
    ASSERT_NEAR(
        nl(NormalLikelihood2d::Vector(0.0, -3.0 * sigma)), likelihood0 * e2b, 1e-6);

    // Check integrated value
    double sum(0.0);
    const double step = 0.1;
    const double step_sq = step * step;
    for (double i = -100.0; i < 100.0; i += step)
    {
      for (double j = -100.0; j < 100.0; j += step)
      {
        sum += nl(NormalLikelihood2d::Vector(i, j)) * step_sq;
      }
    }
    ASSERT_NEAR(sum, 1.0, 1e-6);
  }
  for (double sigma = 1.0; sigma <= 3.0; sigma += 1.0)
  {
    // Check distribution
    using NormalLikelihood2d = mcl_3dl::NormalLikelihoodNd<double, 2>;
    NormalLikelihood2d::Matrix cov;
    cov << sigma, 0,
        0, sigma;
    NormalLikelihood2d nl(cov);

    const double likelihood0 = 1.0 / (M_PI * 2.0 * sqrt(cov.determinant()));
    const double e1 =
        exp(-0.5 *
            NormalLikelihood2d::Vector(sigma, 0.0).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(sigma, 0.0));
    const double e2 =
        exp(-0.5 *
            NormalLikelihood2d::Vector(3.0 * sigma, 0.0).transpose() *
            cov.inverse() *
            NormalLikelihood2d::Vector(3.0 * sigma, 0.0));
    for (double r = 0; r < M_PI * 2; r += M_PI / 6)
    {
      const auto v1 =
          Eigen::Rotation2D<double>(r) *
          NormalLikelihood2d::Vector(sigma, 0.0);
      ASSERT_NEAR(nl(v1), likelihood0 * e1, 1e-6);
      const auto v2 =
          Eigen::Rotation2D<double>(r) *
          NormalLikelihood2d::Vector(3.0 * sigma, 0.0);
      ASSERT_NEAR(nl(v2), likelihood0 * e2, 1e-6);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

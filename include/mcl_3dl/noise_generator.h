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

#ifndef MCL_3DL_NOISE_GENERATOR_H
#define MCL_3DL_NOISE_GENERATOR_H

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "ros/ros.h"

namespace mcl_3dl
{
template <typename FLT_TYPE>
class DiagonalNoiseGenerator
{
public:
  template <typename T>
  explicit DiagonalNoiseGenerator(T& sigma)
    : sigma_(sigma.size())
  {
    for (size_t i = 0; i < sigma.size(); ++i)
    {
      sigma_[i] = sigma[i];
    }
  }

  template <typename RANDOM_ENGINE>
  std::vector<FLT_TYPE> operator()(RANDOM_ENGINE& engine) const
  {
    std::vector<FLT_TYPE> noise(sigma_.size());
    for (size_t i = 0; i < sigma_.size(); i++)
    {
      std::normal_distribution<FLT_TYPE> nd(0.0, sigma_[i]);
      noise[i] = nd(engine);
    }
    return noise;
  }

  size_t getDimension() const
  {
    return sigma_.size();
  }

protected:
  std::vector<FLT_TYPE> sigma_;
};

template <typename FLT_TYPE>
class MultivariateNoiseGenerator
{
public:
  typedef Eigen::Matrix<FLT_TYPE, Eigen::Dynamic, Eigen::Dynamic> Matrix;
  typedef Eigen::Matrix<FLT_TYPE, Eigen::Dynamic, 1> Vector;

  template <typename T>
  explicit MultivariateNoiseGenerator(T& covariances)
    : dim_(std::lround(std::sqrt(covariances.size())))
  {
    Matrix cov_matrix(dim_, dim_);
    for (size_t i = 0; i < dim_; ++i)
    {
      for (size_t j = 0; j < dim_; ++j)
      {
        cov_matrix(i, j) = covariances[j + i * dim_];
      }
    }
    const Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(cov_matrix);
    norm_transform_ = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  template <typename RANDOM_ENGINE>
  std::vector<FLT_TYPE> operator()(RANDOM_ENGINE& engine) const
  {
    Vector random(dim_);
    std::normal_distribution<FLT_TYPE> nd(0.0, 1.0);
    for (int j = 0; j < dim_; ++j)
    {
      random[j] = nd(engine);
    }
    const Vector sample_noise = norm_transform_ * random;
    return std::vector<FLT_TYPE>(sample_noise.data(), sample_noise.data() + dim_);
  }

  size_t getDimension() const
  {
    return dim_;
  }

protected:
  size_t dim_;
  Matrix norm_transform_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_NOISE_GENERATOR_H

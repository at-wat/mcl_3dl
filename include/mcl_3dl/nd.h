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

#ifndef MCL_3DL_ND_H
#define MCL_3DL_ND_H

#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>

namespace mcl_3dl
{
template <typename FLT_TYPE = float>
class NormalLikelihood
{
public:
  explicit NormalLikelihood(const FLT_TYPE sigma)
  {
    a_ = 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma);
    sq2_ = sigma * sigma * 2.0;
  }
  FLT_TYPE operator()(const FLT_TYPE x) const
  {
    return a_ * expf(-x * x / sq2_);
  }

protected:
  FLT_TYPE a_;
  FLT_TYPE sq2_;
};

template <typename FLT_TYPE = float, size_t DIMENSION = 6>
class NormalLikelihoodNd
{
public:
  using Matrix = Eigen::Matrix<FLT_TYPE, DIMENSION, DIMENSION>;
  using Vector = Eigen::Matrix<FLT_TYPE, DIMENSION, 1>;

  explicit NormalLikelihoodNd(const Matrix sigma)
  {
    a_ = 1.0 / (std::pow(2.0 * M_PI, 0.5 * DIMENSION) * std::sqrt(sigma.determinant()));
    sigma_inv_ = sigma.inverse();
  }
  FLT_TYPE operator()(const Vector x) const
  {
    return a_ * std::exp(static_cast<FLT_TYPE>(-0.5 * x.transpose() * sigma_inv_ * x));
  }

protected:
  FLT_TYPE a_;
  Matrix sigma_inv_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_ND_H

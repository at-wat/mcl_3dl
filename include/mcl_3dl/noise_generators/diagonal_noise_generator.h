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

#ifndef MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H
#define MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H

#include <random>
#include <vector>

#include <mcl_3dl/noise_generator_base.h>

namespace mcl_3dl
{
template <typename FLT_TYPE>
class DiagonalNoiseGenerator : public NoiseGeneratorBase<FLT_TYPE>
{
public:
  using Parent = NoiseGeneratorBase<FLT_TYPE>;

  template <typename T>
  DiagonalNoiseGenerator(const T& mean, const T& sigma)
  {
    Parent::setMean(mean);
    setSigma(sigma);
  }

  template <typename T>
  void setSigma(const T& sigma)
  {
    sigma_.resize(sigma.size());
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
      if (sigma_[i] == 0)
      {
        noise[i] = Parent::mean_[i];
        continue;
      }
      std::normal_distribution<FLT_TYPE> nd(Parent::mean_[i], sigma_[i]);
      noise[i] = nd(engine);
    }
    return noise;
  }

protected:
  std::vector<FLT_TYPE> sigma_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_NOISE_GENERATORS_DIAGONAL_NOISE_GENERATOR_H

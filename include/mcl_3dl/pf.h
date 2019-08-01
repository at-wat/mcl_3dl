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

#ifndef MCL_3DL_PF_H
#define MCL_3DL_PF_H

#include <random>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>

namespace mcl_3dl
{
namespace pf
{
template <typename FLT_TYPE = float>
class ParticleBase
{
public:
  virtual FLT_TYPE& operator[](const size_t i) = 0;
  virtual size_t size() const = 0;
  virtual void normalize() = 0;
  template <typename T>
  T operator+(const T& a)
  {
    T in = a;
    T ret;
    for (size_t i = 0; i < size(); i++)
    {
      ret[i] = (*this)[i] + in[i];
    }
    return ret;
  }
  template <typename T>
  FLT_TYPE covElement(
      const T& e, const size_t& j, const size_t& k)
  {
    T exp = e;
    return ((*this)[k] - exp[k]) * ((*this)[j] - exp[j]);
  }
  template <typename T>
  static T generateNoise(
      std::default_random_engine& engine_,
      T mean, T sigma)
  {
    T noise;
    for (size_t i = 0; i < noise.size(); i++)
    {
      std::normal_distribution<FLT_TYPE> nd(mean[i], sigma[i]);
      noise[i] = nd(engine_);
    }
    return noise;
  }

protected:
};

template <typename T, typename FLT_TYPE = float>
class Particle
{
public:
  Particle()
  {
    probability_ = 0.0;
    probability_bias_ = 0.0;
  }
  explicit Particle(FLT_TYPE prob)
  {
    accum_probability_ = prob;
  }
  T state_;
  FLT_TYPE probability_;
  FLT_TYPE probability_bias_;
  FLT_TYPE accum_probability_;
  bool operator<(const Particle& p2) const
  {
    return this->accum_probability_ < p2.accum_probability_;
  }
};

template <typename T, typename FLT_TYPE = float>
class ParticleWeightedMean
{
protected:
  T e_;
  FLT_TYPE p_sum_;

public:
  ParticleWeightedMean()
    : e_()
    , p_sum_(0.0)
  {
  }

  void add(const T& s, const FLT_TYPE& prob)
  {
    p_sum_ += prob;

    T e1 = s;
    for (size_t i = 0; i < e1.size(); i++)
    {
      e1[i] = e1[i] * prob;
    }
    e_ = e1 + e_;
  }

  T getMean()
  {
    assert(p_sum_ > 0.0);

    T s = e_;

    for (size_t i = 0; i < s.size(); i++)
    {
      s[i] = s[i] / p_sum_;
    }

    return s;
  }

  FLT_TYPE getTotalProbability()
  {
    return p_sum_;
  }
};

template <typename T, typename FLT_TYPE = float, typename MEAN = ParticleWeightedMean<T, FLT_TYPE>>
class ParticleFilter
{
public:
  explicit ParticleFilter(const int num_particles)
    : engine_(seed_gen_())
  {
    particles_.resize(num_particles);
  }
  void init(T mean, T sigma)
  {
    for (auto& p : particles_)
    {
      p.state_ = T::generateNoise(engine_, mean, sigma);
      p.probability_ = 1.0 / particles_.size();
    }
  }
  void resample(T sigma)
  {
    FLT_TYPE accum = 0;
    for (auto& p : particles_)
    {
      accum += p.probability_;
      p.accum_probability_ = accum;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());

    const FLT_TYPE pstep = accum / particles_.size();
    auto it = particles_dup_.begin();
    auto it_prev = particles_dup_.begin();
    const FLT_TYPE prob = 1.0 / particles_.size();
    for (size_t i = 0; i < particles_.size(); ++i)
    {
      auto& p = particles_[i];
      const FLT_TYPE pscan = std::nextafter(pstep * (i + 1), static_cast<FLT_TYPE>(0.0));
      it = std::lower_bound(it, particles_dup_.end(), Particle<T, FLT_TYPE>(pscan));
      p.probability_ = prob;
      if (it == particles_dup_.end())
      {
        p.state_ = it_prev->state_;
        continue;
      }
      else if (it == it_prev)
      {
        p.state_ = it->state_ + T::generateNoise(engine_, T(), sigma);
        p.state_.normalize();
      }
      else
      {
        p.state_ = it->state_;
      }
      it_prev = it;
    }
  }
  void noise(T sigma)
  {
    for (auto& p : particles_)
    {
      p.state_ = p.state_ + T::generateNoise(engine_, T(), sigma);
    }
  }
  void predict(std::function<void(T&)> model)
  {
    for (auto& p : particles_)
    {
      model(p.state_);
    }
  }
  void bias(std::function<void(const T&, float& p_bias)> prob)
  {
    for (auto& p : particles_)
    {
      prob(p.state_, p.probability_bias_);
    }
  }
  void measure(std::function<FLT_TYPE(const T&)> likelihood)
  {
    auto particles_prev = particles_;  // backup old
    FLT_TYPE sum = 0;
    for (auto& p : particles_)
    {
      p.probability_ *= likelihood(p.state_);
      sum += p.probability_;
    }
    if (sum > 0.0)
    {
      for (auto& p : particles_)
      {
        p.probability_ /= sum;
      }
    }
    else
    {
      particles_ = particles_prev;
      // std::cerr << "No Particle alive, restoring." << std::endl;
    }
  }
  T expectation(const FLT_TYPE pass_ratio = 1.0)
  {
    MEAN mean;

    if (pass_ratio < 1.0)
      std::sort(particles_.rbegin(), particles_.rend());
    for (auto& p : particles_)
    {
      mean.add(p.state_, p.probability_);
      if (mean.getTotalProbability() > pass_ratio)
        break;
    }
    return mean.getMean();
  }
  T expectationBiased()
  {
    MEAN mean;

    for (auto& p : particles_)
    {
      mean.add(p.state_, p.probability_ * p.probability_bias_);
    }
    return mean.getMean();
  }
  std::vector<T> covariance(
      const FLT_TYPE pass_ratio = 1.0,
      const FLT_TYPE random_sample_ratio = 1.0)
  {
    T e = expectation(pass_ratio);
    FLT_TYPE p_sum = 0;
    std::vector<T> cov;
    cov.resize(e.size());

    size_t p_num = 0;
    for (auto& p : particles_)
    {
      p_num++;
      p_sum += p.probability_;
      if (p_sum > pass_ratio)
        break;
    }

    const size_t num =
        std::min(
            p_num,
            std::max(
                size_t(0),
                static_cast<size_t>(p_num * random_sample_ratio)));
    std::vector<size_t> full_indices(p_num);
    std::iota(full_indices.begin(), full_indices.end(), 0);
    std::vector<size_t> indices;
    indices.reserve(num);
    std::sample(
        full_indices.begin(), full_indices.end(),
        std::back_inserter(indices), num, engine_);

    p_sum = 0.0;
    for (size_t i : indices)
    {
      auto& p = particles_[i];
      p_sum += p.probability_;
      for (size_t j = 0; j < ie_.size(); j++)
      {
        for (size_t k = j; k < ie_.size(); k++)
        {
          cov[k][j] = cov[j][k] += p.state_.covElement(e, j, k) * p.probability_;
        }
      }
    }
    std::cerr << particles_.size() << "/" << p_num << "/" << indices.size() << " (" << p_sum << std::endl;
    for (size_t j = 0; j < ie_.size(); j++)
    {
      for (size_t k = 0; k < ie_.size(); k++)
      {
        cov[k][j] /= p_sum;
      }
    }

    return cov;
  }
  T max()
  {
    T* m = &particles_[0].state_;
    FLT_TYPE max_probability = particles_[0].probability_;
    for (auto& p : particles_)
    {
      if (max_probability < p.probability_)
      {
        max_probability = p.probability_;
        m = &p.state_;
      }
    }
    return *m;
  }
  T maxBiased()
  {
    T* m = &particles_[0].state_;
    FLT_TYPE max_probability =
        particles_[0].probability_ * particles_[0].probability_bias_;
    for (auto& p : particles_)
    {
      const FLT_TYPE prob = p.probability_ * p.probability_bias_;
      if (max_probability < prob)
      {
        max_probability = prob;
        m = &p.state_;
      }
    }
    return *m;
  }
  T getParticle(const size_t i) const
  {
    return particles_[i].state_;
  }
  size_t getParticleSize() const
  {
    return particles_.size();
  }
  void resizeParticle(const size_t num)
  {
    FLT_TYPE accum = 0;
    for (auto& p : particles_)
    {
      accum += p.probability_;
      p.accum_probability_ = accum;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());

    FLT_TYPE pstep = accum / num;
    FLT_TYPE pscan = 0;
    auto it = particles_dup_.begin();
    auto it_prev = particles_dup_.begin();

    particles_.resize(num);

    FLT_TYPE prob = 1.0 / num;
    for (auto& p : particles_)
    {
      pscan += pstep;
      it = std::lower_bound(it, particles_dup_.end(),
                            Particle<T, FLT_TYPE>(pscan));
      p.probability_ = prob;
      if (it == particles_dup_.end())
      {
        p.state_ = it_prev->state_;
        continue;
      }
      else
      {
        p.state_ = it->state_;
      }
      it_prev = it;
    }
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator appendParticle(const size_t num)
  {
    const size_t size_orig = particles_.size();
    particles_.resize(size_orig + num);
    return begin() + size_orig;
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator begin()
  {
    return particles_.begin();
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator end()
  {
    return particles_.end();
  }

protected:
  std::vector<Particle<T, FLT_TYPE>> particles_;
  std::vector<Particle<T, FLT_TYPE>> particles_dup_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  T ie_;
};

}  // namespace pf
}  // namespace mcl_3dl

#endif  // MCL_3DL_PF_H

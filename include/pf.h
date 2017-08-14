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

#ifndef PF_H
#define PF_H

#include <random>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>

namespace pf
{
template <typename FLT_TYPE = float>
class ParticleBase
{
public:
  virtual FLT_TYPE &operator[](const size_t i) = 0;
  virtual size_t size() const = 0;
  virtual void normalize() = 0;
  template <typename T>
  T operator+(const T &a)
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
      const T &e, const size_t &j, const size_t &k)
  {
    T exp = e;
    return ((*this)[k] - exp[k]) * ((*this)[j] - exp[j]);
  }
  template <typename T>
  T generateNoise(
      std::default_random_engine &engine_,
      T mean, T sigma)
  {
    T noise;
    for (size_t i = 0; i < size(); i++)
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
    probability = 0.0;
    probability_bias = 0.0;
  }
  explicit Particle(FLT_TYPE prob)
  {
    accum_probability = prob;
  }
  T state;
  FLT_TYPE probability;
  FLT_TYPE probability_bias;
  FLT_TYPE accum_probability;
  bool operator<(const Particle &p2) const
  {
    return this->accum_probability < p2.accum_probability;
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

  void add(const T &s, const FLT_TYPE &prob)
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
  explicit ParticleFilter(const int nParticles)
    : engine_(seed_gen_())
  {
    particles_.resize(nParticles);
  }
  void init(T mean, T sigma)
  {
    for (auto &p : particles_)
    {
      p.state = p.state.generateNoise(engine_, mean, sigma);
      p.probability = 1.0 / particles_.size();
    }
  }
  void resample(T sigma)
  {
    FLT_TYPE accum = 0;
    for (auto &p : particles_)
    {
      accum += p.probability;
      p.accum_probability = accum;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());

    FLT_TYPE pstep = accum / particles_.size();
    FLT_TYPE pscan = 0;
    auto it = particles_dup_.begin();
    auto it_prev = particles_dup_.begin();

    FLT_TYPE prob = 1.0 / particles_.size();
    for (auto &p : particles_)
    {
      pscan += pstep;
      it = std::lower_bound(it, particles_dup_.end(), Particle<T, FLT_TYPE>(pscan));
      if (it == particles_dup_.end())
      {
        p.state = it_prev->state;
      }
      else if (it == it_prev)
      {
        p.state = it->state + it->state.generateNoise(engine_, T(), sigma);
        p.state.normalize();
      }
      else
      {
        p.state = it->state;
      }
      it_prev = it;
      p.probability = prob;
    }
  }
  void noise(T sigma)
  {
    for (auto &p : particles_)
    {
      p.state = p.state + p.state.generateNoise(engine_, T(), sigma);
    }
  }
  void predict(std::function<void(T &)> model)
  {
    for (auto &p : particles_)
    {
      model(p.state);
    }
  }
  void bias(std::function<void(const T &, float &p_bias)> prob)
  {
    for (auto &p : particles_)
    {
      prob(p.state, p.probability_bias);
    }
  }
  void measure(std::function<FLT_TYPE(const T &)> likelihood)
  {
    auto particles_prev = particles_;  // backup old
    FLT_TYPE sum = 0;
    for (auto &p : particles_)
    {
      p.probability *= likelihood(p.state);
      sum += p.probability;
    }
    if (sum > 0.0)
    {
      for (auto &p : particles_)
      {
        p.probability /= sum;
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
    for (auto &p : particles_)
    {
      mean.add(p.state, p.probability);
      if (mean.getTotalProbability() > pass_ratio)
        break;
    }
    return mean.getMean();
  }
  T expectationBiased()
  {
    MEAN mean;

    for (auto &p : particles_)
    {
      mean.add(p.state, p.probability * p.probability_bias);
    }
    return mean.getMean();
  }
  std::vector<T> covariance(const FLT_TYPE pass_ratio = 1.0)
  {
    T e = expectation(pass_ratio);
    FLT_TYPE p_sum = 0;
    size_t p_num = 0;
    std::vector<T> cov;
    cov.resize(e.size());

    for (auto &p : particles_)
    {
      p_num++;
      p_sum += p.probability;
      if (p_sum > pass_ratio)
        break;
    }
    p_sum = 0;
    for (auto &p : particles_)
    {
      for (size_t j = 0; j < ie_.size(); j++)
      {
        for (size_t k = j; k < ie_.size(); k++)
        {
          cov[k][j] = cov[j][k] += p.state.covElement(e, j, k) * p.probability;
        }
      }

      p_sum += p.probability;
      if (p_sum > pass_ratio)
        break;
    }
    for (size_t j = 0; j < ie_.size(); j++)
    {
      for (size_t k = j; k < ie_.size(); k++)
      {
        cov[k][j] /= p_sum;
        cov[j][k] /= p_sum;
      }
    }

    return cov;
  }
  T max()
  {
    T *m = &particles_[0].state;
    FLT_TYPE max_probability = particles_[0].probability;
    for (auto &p : particles_)
    {
      if (max_probability < p.probability)
      {
        max_probability = p.probability;
        m = &p.state;
      }
    }
    return *m;
  }
  T maxBiased()
  {
    T *m = &particles_[0].state;
    FLT_TYPE max_probability =
        particles_[0].probability * particles_[0].probability_bias;
    for (auto &p : particles_)
    {
      const FLT_TYPE prob = p.probability * p.probability_bias;
      if (max_probability < prob)
      {
        max_probability = prob;
        m = &p.state;
      }
    }
    return *m;
  }
  T getParticle(const size_t i) const
  {
    return particles_[i].state;
  }
  size_t getParticleSize() const
  {
    return particles_.size();
  }
  void resizeParticle(const size_t num)
  {
    FLT_TYPE accum = 0;
    for (auto &p : particles_)
    {
      accum += p.probability;
      p.accum_probability = accum;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());

    FLT_TYPE pstep = accum / particles_.size();
    FLT_TYPE pscan = 0;
    auto it = particles_dup_.begin();

    particles_.resize(num);
    FLT_TYPE prob = 1.0 / num;
    for (auto &p : particles_)
    {
      pscan += pstep;
      it = std::lower_bound(it, particles_dup_.end(), Particle<T, FLT_TYPE>(pscan));
      p.state = it->state;
      p.probability = prob;
    }
  }

protected:
  std::vector<Particle<T, FLT_TYPE>> particles_;
  std::vector<Particle<T, FLT_TYPE>> particles_dup_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  T ie_;
};

}  // namespace pf

#endif  // PF_H

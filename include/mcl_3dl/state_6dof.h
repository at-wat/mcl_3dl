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

#ifndef MCL_3DL_STATE_6DOF_H
#define MCL_3DL_STATE_6DOF_H

#include <algorithm>
#include <cassert>
#include <vector>

#include <ros/ros.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/vec3.h>

#include <mcl_3dl/noise_generator_base.h>
#include <mcl_3dl/noise_generators/diagonal_noise_generator.h>

namespace mcl_3dl
{
class State6DOF : public mcl_3dl::pf::ParticleBase<float>
{
public:
  mcl_3dl::Vec3 pos_;
  mcl_3dl::Quat rot_;
  bool diff_;
  float noise_ll_;
  float noise_la_;
  float noise_al_;
  float noise_aa_;
  mcl_3dl::Vec3 odom_err_integ_lin_;
  mcl_3dl::Vec3 odom_err_integ_ang_;
  class RPYVec
  {
  public:
    mcl_3dl::Vec3 v_;
    RPYVec()
    {
    }
    explicit RPYVec(const mcl_3dl::Vec3& v)
    {
      v_ = v;
    }
    RPYVec(const float r, const float p, const float y)
    {
      v_.x_ = r;
      v_.y_ = p;
      v_.z_ = y;
    }
  };
  RPYVec rpy_;
  float& operator[](const size_t i) override
  {
    switch (i)
    {
      case 0:
        return pos_.x_;
      case 1:
        return pos_.y_;
      case 2:
        return pos_.z_;
      case 3:
        return rot_.x_;
      case 4:
        return rot_.y_;
      case 5:
        return rot_.z_;
      case 6:
        return rot_.w_;
      case 7:
        return odom_err_integ_lin_.x_;
      case 8:
        return odom_err_integ_lin_.y_;
      case 9:
        return odom_err_integ_lin_.z_;
      case 10:
        return odom_err_integ_ang_.x_;
      case 11:
        return odom_err_integ_ang_.y_;
      case 12:
        return odom_err_integ_ang_.z_;
      default:
        assert(false);
    }
    return pos_.x_;
  }
  float operator[](const size_t i) const
  {
    switch (i)
    {
      case 0:
        return pos_.x_;
      case 1:
        return pos_.y_;
      case 2:
        return pos_.z_;
      case 3:
        return rot_.x_;
      case 4:
        return rot_.y_;
      case 5:
        return rot_.z_;
      case 6:
        return rot_.w_;
      case 7:
        return odom_err_integ_lin_.x_;
      case 8:
        return odom_err_integ_lin_.y_;
      case 9:
        return odom_err_integ_lin_.z_;
      case 10:
        return odom_err_integ_ang_.x_;
      case 11:
        return odom_err_integ_ang_.y_;
      case 12:
        return odom_err_integ_ang_.z_;
      default:
        assert(false);
    }
    return 0;
  }
  size_t size() const override
  {
    return 13;
  }
  void normalize() override
  {
    rot_.normalize();
  }
  size_t covDimension() const override
  {
    return 6;
  }
  float covElement(const State6DOF& e, const size_t& j, const size_t& k)
  {
    const mcl_3dl::Vec3 exp_rpy = e.isDiff() ? e.rpy_.v_ : e.rot_.getRPY();
    const mcl_3dl::Vec3 rpy = isDiff() ? rpy_.v_ : rot_.getRPY();
    float val = 1.0f, diff = 0.0f;
    for (size_t i : {j, k})
    {
      if (i < 3)
      {
        diff = (*this)[i] - e[i];
      }
      else
      {
        diff = rpy[i - 3] - exp_rpy[i - 3];
        while (diff > M_PI)
          diff -= 2 * M_PI;
        while (diff < -M_PI)
          diff += 2 * M_PI;
      }
      val *= diff;
    }
    return val;
  }
  State6DOF()
  {
    diff_ = false;
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
  }
  State6DOF(const mcl_3dl::Vec3 pos, const mcl_3dl::Quat rot)
  {
    pos_ = pos;
    rot_ = rot;
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    diff_ = false;
  }
  State6DOF(const mcl_3dl::Vec3 pos, const mcl_3dl::Vec3 rpy)
  {
    pos_ = pos;
    rpy_ = RPYVec(rpy);
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang_ = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    diff_ = true;
  }
  bool isDiff() const
  {
    return diff_;
  }
  template <typename PointType>
  void transform(pcl::PointCloud<PointType>& pc) const
  {
    const auto r = rot_.normalized();
    for (auto& p : pc.points)
    {
      const Vec3 t = r * Vec3(p.x, p.y, p.z) + pos_;
      p.x = t.x_;
      p.y = t.y_;
      p.z = t.z_;
    }
  }
  template <typename T, typename RANDOM_ENGINE, typename NOISE_GEN>
  static State6DOF generateNoise(RANDOM_ENGINE& engine, const NOISE_GEN& gen)
  {
    if (gen.getDimension() != 6)
    {
      ROS_ERROR("Dimension of noise must be 6. Passed: %lu", gen.getDimension());
    }
    State6DOF noise;
    const std::vector<float> org_noise = gen(engine);
    const std::vector<float>& mean = gen.getMean();
    for (size_t i = 0; i < 3; i++)
    {
      noise[i] = noise[i + 7] = org_noise[i];
    }
    mcl_3dl::Vec3 rpy_noise;
    for (size_t i = 0; i < 3; i++)
    {
      rpy_noise[i] = org_noise[i + 3];
      noise[i + 10] = org_noise[i + 3] - mean[i + 3];
    }
    noise.rot_ = mcl_3dl::Quat(rpy_noise);
    return noise;
  }
  State6DOF operator+(const State6DOF& a) const
  {
    State6DOF in = a;
    State6DOF ret;
    for (size_t i = 0; i < size(); i++)
    {
      if (3 <= i && i <= 6)
        continue;
      ret[i] = (*this)[i] + in[i];
    }
    ret.rot_ = a.rot_ * rot_;
    return ret;
  }
  State6DOF operator-(const State6DOF& a) const
  {
    State6DOF in = a;
    State6DOF ret;
    for (size_t i = 0; i < size(); i++)
    {
      if (3 <= i && i <= 6)
        continue;
      ret[i] = (*this)[i] - in[i];
    }
    ret.rot_ = a.rot_.inv() * rot_;
    return ret;
  }
};

template <>
template <>
inline void NoiseGeneratorBase<float>::setMean(const State6DOF& mean)
{
  mean_.resize(6);
  if (mean.isDiff())
  {
    ROS_ERROR("Failed to generate noise. mean must be mcl_3dl::Quat.");
  }
  for (size_t i = 0; i < 3; i++)
  {
    mean_[i] = mean[i];
  }
  const Vec3 rpy = mean.rot_.getRPY();
  for (size_t i = 0; i < 3; i++)
  {
    mean_[i + 3] = rpy[i];
  }
}

template <>
template <>
inline void DiagonalNoiseGenerator<float>::setSigma(const State6DOF& sigma)
{
  sigma_.resize(6);
  if (!sigma.isDiff())
  {
    ROS_ERROR("Failed to generate noise. sigma must be rpy vec.");
  }
  for (size_t i = 0; i < 3; i++)
  {
    sigma_[i] = sigma[i];
  }
  for (size_t i = 0; i < 3; i++)
  {
    sigma_[i + 3] = sigma.rpy_.v_[i];
  }
}

class ParticleWeightedMeanQuat : public mcl_3dl::pf::ParticleWeightedMean<State6DOF, float>
{
protected:
  mcl_3dl::Vec3 front_sum_;
  mcl_3dl::Vec3 up_sum_;

public:
  ParticleWeightedMeanQuat()
    : ParticleWeightedMean()
    , front_sum_(0.0, 0.0, 0.0)
    , up_sum_(0.0, 0.0, 0.0)
  {
  }

  void add(const State6DOF& s, const float& prob)
  {
    p_sum_ += prob;

    State6DOF e1 = s;
    e_.pos_ += e1.pos_ * prob;

    const mcl_3dl::Vec3 front = s.rot_ * mcl_3dl::Vec3(1.0, 0.0, 0.0) * prob;
    const mcl_3dl::Vec3 up = s.rot_ * mcl_3dl::Vec3(0.0, 0.0, 1.0) * prob;

    front_sum_ += front;
    up_sum_ += up;
  }

  State6DOF getMean()
  {
    assert(p_sum_ > 0.0);

    return State6DOF(e_.pos_ / p_sum_, mcl_3dl::Quat(front_sum_, up_sum_));
  }

  float getTotalProbability()
  {
    return p_sum_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_STATE_6DOF_H

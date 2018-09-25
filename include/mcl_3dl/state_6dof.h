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

#include <mcl_3dl/pf.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class State6DOF : public mcl_3dl::pf::ParticleBase<float>
{
public:
  mcl_3dl::Vec3 pos;
  mcl_3dl::Quat rot;
  bool diff;
  float noise_ll_;
  float noise_la_;
  float noise_al_;
  float noise_aa_;
  mcl_3dl::Vec3 odom_err_integ_lin;
  mcl_3dl::Vec3 odom_err_integ_ang;
  class RPYVec
  {
  public:
    mcl_3dl::Vec3 v;
    RPYVec()
    {
    }
    explicit RPYVec(const mcl_3dl::Vec3 &v)
    {
      this->v = v;
    }
    RPYVec(const float &r, const float &p, const float y)
    {
      this->v.x = r;
      this->v.y = p;
      this->v.z = y;
    }
  };
  RPYVec rpy;
  float &operator[](const size_t i)override
  {
    switch (i)
    {
      case 0:
        return pos.x;
      case 1:
        return pos.y;
      case 2:
        return pos.z;
      case 3:
        return rot.x;
      case 4:
        return rot.y;
      case 5:
        return rot.z;
      case 6:
        return rot.w;
      case 7:
        return odom_err_integ_lin.x;
      case 8:
        return odom_err_integ_lin.y;
      case 9:
        return odom_err_integ_lin.z;
      case 10:
        return odom_err_integ_ang.x;
      case 11:
        return odom_err_integ_ang.y;
      case 12:
        return odom_err_integ_ang.z;
    }
    return pos.x;
  }
  float operator[](const size_t i) const
  {
    switch (i)
    {
      case 0:
        return pos.x;
      case 1:
        return pos.y;
      case 2:
        return pos.z;
      case 3:
        return rot.x;
      case 4:
        return rot.y;
      case 5:
        return rot.z;
      case 6:
        return rot.w;
      case 7:
        return odom_err_integ_lin.x;
      case 8:
        return odom_err_integ_lin.y;
      case 9:
        return odom_err_integ_lin.z;
      case 10:
        return odom_err_integ_ang.x;
      case 11:
        return odom_err_integ_ang.y;
      case 12:
        return odom_err_integ_ang.z;
    }
    return pos.x;
  }
  size_t size() const override
  {
    return 10;
  }
  void normalize() override
  {
    rot.normalize();
  }
  State6DOF()
  {
    diff = false;
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang = mcl_3dl::Vec3(0.0, 0.0, 0.0);
  }
  State6DOF(const mcl_3dl::Vec3 pos, const mcl_3dl::Quat rot)
  {
    this->pos = pos;
    this->rot = rot;
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    diff = false;
  }
  State6DOF(const mcl_3dl::Vec3 pos, const mcl_3dl::Vec3 rpy)
  {
    this->pos = pos;
    this->rpy = RPYVec(rpy);
    noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
    odom_err_integ_lin = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    odom_err_integ_ang = mcl_3dl::Vec3(0.0, 0.0, 0.0);
    diff = true;
  }
  bool isDiff() const
  {
    return diff;
  }
  void transform(pcl::PointCloud<pcl::PointXYZI> &pc) const
  {
    auto r = rot.normalized();
    for (auto &p : pc.points)
    {
      auto t = r * mcl_3dl::Vec3(p.x, p.y, p.z) + pos;
      p.x = t.x;
      p.y = t.y;
      p.z = t.z;
    }
  }
  static State6DOF generateNoise(
      std::default_random_engine &engine_,
      const State6DOF &mean, const State6DOF &sigma)
  {
    State6DOF noise;
    if (mean.isDiff() || !sigma.isDiff())
    {
      ROS_ERROR("Failed to generate noise. mean must be mcl_3dl::Quat and sigma must be rpy vec.");
    }
    for (size_t i = 0; i < 3; i++)
    {
      std::normal_distribution<float> nd(mean[i], sigma[i]);
      noise[i] = noise[i + 7] = nd(engine_);
    }
    mcl_3dl::Vec3 rpy_noise;
    for (size_t i = 0; i < 3; i++)
    {
      std::normal_distribution<float> nd(0.0, sigma.rpy.v[i]);
      rpy_noise[i] = noise[i + 10] = nd(engine_);
    }
    noise.rot = mcl_3dl::Quat(rpy_noise) * mean.rot;
    return noise;
  }
  State6DOF operator+(const State6DOF &a) const
  {
    State6DOF in = a;
    State6DOF ret;
    for (size_t i = 0; i < size(); i++)
    {
      if (3 <= i && i <= 6)
        continue;
      ret[i] = (*this)[i] + in[i];
    }
    ret.rot = a.rot * rot;
    return ret;
  }
  State6DOF operator-(const State6DOF &a) const
  {
    State6DOF in = a;
    State6DOF ret;
    for (size_t i = 0; i < size(); i++)
    {
      if (3 <= i && i <= 6)
        continue;
      ret[i] = (*this)[i] - in[i];
    }
    ret.rot = a.rot * rot.inv();
    return ret;
  }
};
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

  void add(const State6DOF &s, const float &prob)
  {
    p_sum_ += prob;

    State6DOF e1 = s;
    e_.pos += e1.pos * prob;

    const mcl_3dl::Vec3 front = s.rot * mcl_3dl::Vec3(1.0, 0.0, 0.0) * prob;
    const mcl_3dl::Vec3 up = s.rot * mcl_3dl::Vec3(0.0, 0.0, 1.0) * prob;

    front_sum_ += front;
    up_sum_ += up;
  }

  State6DOF getMean()
  {
    assert(p_sum_ > 0.0);

    return State6DOF(e_.pos / p_sum_, mcl_3dl::Quat(front_sum_, up_sum_));
  }

  float getTotalProbability()
  {
    return p_sum_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_STATE_6DOF_H

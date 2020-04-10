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

#ifndef MCL_3DL_FILTER_H
#define MCL_3DL_FILTER_H

#include <cassert>
#include <cmath>

namespace mcl_3dl
{
class Filter
{
public:
  enum type_t
  {
    FILTER_HPF,
    FILTER_LPF
  };

protected:
  bool angle_;
  float x_;
  float out_;
  float k_[4];

public:
  inline Filter(const enum type_t type, const float time_const, const float out0, const bool angle = false)
    : angle_(angle)
    , out_(out0)
  {
    switch (type)
    {
      case FILTER_LPF:
        k_[3] = -1 / (1.0 + 2 * time_const);
        k_[2] = -k_[3];
        k_[1] = (1.0 - 2 * time_const) * k_[3];
        k_[0] = -k_[1] - 1.0;
        x_ = (1 - k_[2]) * out0 / k_[3];
        break;
      case FILTER_HPF:
        k_[3] = -1 / (1.0 + 2 * time_const);
        k_[2] = -k_[3] * 2 * time_const;
        k_[1] = (1.0 - 2 * time_const) * k_[3];
        k_[0] = 2 * time_const * (-k_[1] + 1.0);
        x_ = (1 - k_[2]) * out0 / k_[3];
        break;
    }
  }
  inline void set(const float out0)
  {
    x_ = (1 - k_[2]) * out0 / k_[3];
    out_ = out0;
  }
  inline float in(float in)
  {
    assert(std::isfinite(in));

    if (angle_)
    {
      in = out_ + remainder(in - out_, M_PI * 2.0);
    }
    x_ = k_[0] * in + k_[1] * x_;
    out_ = k_[2] * in + k_[3] * x_;

    assert(std::isfinite(out_));
    return out_;
  }
  inline float get() const
  {
    return out_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_FILTER_H

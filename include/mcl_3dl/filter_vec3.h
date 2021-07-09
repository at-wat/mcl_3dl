/*
 * Copyright (c) 2020, the mcl_3dl authors
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

#ifndef MCL_3DL_FILTER_VEC3_H
#define MCL_3DL_FILTER_VEC3_H

#include <array>

#include <mcl_3dl/filter.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class FilterVec3
{
private:
  Vec3 x_;
  std::array<Filter, 3> f_;

public:
  inline FilterVec3(Filter::type_t type, const Vec3& time_const, const Vec3& out0, const bool angle = false)
    : x_(out0)
    , f_(
          {
              Filter(type, time_const[0], out0[0], angle),
              Filter(type, time_const[1], out0[1], angle),
              Filter(type, time_const[2], out0[2], angle),
          }  //
      )
  {
  }
  inline void set(const Vec3& out0)
  {
    x_ = out0;
    for (int i = 0; i < 3; i++)
      f_[i].set(out0[i]);
  }
  inline Vec3 in(const Vec3& in)
  {
    for (int i = 0; i < 3; i++)
      x_[i] = f_[i].in(in[i]);
    return x_;
  }
  inline Vec3 get() const
  {
    return x_;
  }
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_FILTER_VEC3_H

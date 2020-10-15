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

#ifndef MCL_3DL_RAYCAST_H
#define MCL_3DL_RAYCAST_H

#include <cmath>
#include <vector>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
template <typename POINT_TYPE>
class Raycast
{
public:
  class CastResult
  {
  public:
    Vec3 pos_;
    bool collision_;
    float sin_angle_;
    const POINT_TYPE* point_;

    CastResult()
      : pos_(0, 0, 0)
      , collision_(false)
      , sin_angle_(0)
      , point_(nullptr)
    {
    }

    CastResult(const Vec3& pos, bool collision, float sin_angle, const POINT_TYPE* point)
      : pos_(pos)
      , collision_(collision)
      , sin_angle_(sin_angle)
      , point_(point)
    {
    }
  };

  Raycast()
  {
  }
  virtual void setRay(typename ChunkedKdtree<POINT_TYPE>::Ptr kdtree, const Vec3 ray_begin, const Vec3 ray_end) = 0;
  virtual bool getNextCastResult(CastResult& result) = 0;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_RAYCAST_H

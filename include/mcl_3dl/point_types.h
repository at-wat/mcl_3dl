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

#ifndef MCL_3DL_POINT_TYPES_H
#define MCL_3DL_POINT_TYPES_H

#include <cstdint>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mcl_3dl
{
struct EIGEN_ALIGN16 PointXYZIL
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t label;

  inline PointXYZIL()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    intensity = 0.0f;
    label = 0;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace mcl_3dl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    mcl_3dl::PointXYZIL,
    (float, x, x)                  //
    (float, y, y)                  //
    (float, z, z)                  //
    (float, intensity, intensity)  //
    (std::uint32_t, label, label))

#endif  // MCL_3DL_POINT_TYPES_H

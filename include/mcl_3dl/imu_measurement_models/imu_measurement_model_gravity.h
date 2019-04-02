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

#ifndef MCL_3DL_IMU_MEASUREMENT_MODELS_IMU_MEASUREMENT_MODEL_GRAVITY_H
#define MCL_3DL_IMU_MEASUREMENT_MODELS_IMU_MEASUREMENT_MODEL_GRAVITY_H

#include <memory>

#include <mcl_3dl/imu_measurement_model_base.h>
#include <mcl_3dl/nd.h>

namespace mcl_3dl
{
class ImuMeasurementModelGravity : public ImuMeasurementModelBase
{
public:
  explicit ImuMeasurementModelGravity(const float acc_var)
    : nd_(acc_var)
  {
  }

  inline void setAccMeasure(const Vec3& acc_measure) final
  {
    acc_measure_ = acc_measure;
    acc_measure_norm_ = acc_measure.norm();
  }

  inline float measure(const State6DOF& s) const final
  {
    const Vec3 acc_estim = s.rot_.inv() * Vec3(0.0, 0.0, 1.0);
    const float diff = acosf(
        acc_estim.dot(acc_measure_) / (acc_measure_norm_ * acc_estim.norm()));
    return nd_(diff);
  }

private:
  NormalLikelihood<float> nd_;
  Vec3 acc_measure_;
  float acc_measure_norm_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_IMU_MEASUREMENT_MODELS_IMU_MEASUREMENT_MODEL_GRAVITY_H

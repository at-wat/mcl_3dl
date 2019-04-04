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

#ifndef MCL_3DL_MOTION_PREDICTION_MODELS_MOTION_PREDICTION_MODEL_DIFFERENTIAL_DRIVE_H
#define MCL_3DL_MOTION_PREDICTION_MODELS_MOTION_PREDICTION_MODEL_DIFFERENTIAL_DRIVE_H

#include <mcl_3dl/motion_prediction_model_base.h>

namespace mcl_3dl
{
class MotionPredictionModelDifferentialDrive : public MotionPredictionModelBase
{
public:
  MotionPredictionModelDifferentialDrive(const float odom_err_integ_lin_tc, const float odom_err_integ_ang_tc)
    : odom_err_integ_lin_tc_(odom_err_integ_lin_tc)
    , odom_err_integ_ang_tc_(odom_err_integ_ang_tc)
  {
  }

  inline void setOdoms(const State6DOF& odom_prev, const State6DOF& odom_current, const float time_diff) final
  {
    relative_translation_ = odom_prev.rot_.inv() * (odom_current.pos_ - odom_prev.pos_);
    relative_quat_ = odom_prev.rot_.inv() * odom_current.rot_;
    Vec3 axis;
    relative_quat_.getAxisAng(axis, relative_angle_);
    relative_translation_norm_ = relative_translation_.norm();
    time_diff_ = time_diff;
  };

  inline void predict(State6DOF& s) const final
  {
    const Vec3 diff = relative_translation_ * (1.0 + s.noise_ll_) + Vec3(s.noise_al_ * relative_angle_, 0.0, 0.0);
    s.odom_err_integ_lin_ += (diff - relative_translation_);
    s.pos_ += s.rot_ * diff;
    const float yaw_diff = s.noise_la_ * relative_translation_norm_ + s.noise_aa_ * relative_angle_;
    s.rot_ = Quat(Vec3(0.0, 0.0, 1.0), yaw_diff) * s.rot_ * relative_quat_;
    s.rot_.normalize();
    s.odom_err_integ_ang_ += Vec3(0.0, 0.0, yaw_diff);
    s.odom_err_integ_lin_ *= (1.0 - time_diff_ / odom_err_integ_lin_tc_);
    s.odom_err_integ_ang_ *= (1.0 - time_diff_ / odom_err_integ_ang_tc_);
  };

private:
  const float odom_err_integ_lin_tc_;
  const float odom_err_integ_ang_tc_;
  Vec3 relative_translation_;
  Quat relative_quat_;
  float relative_translation_norm_;
  float relative_angle_;
  float time_diff_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_MOTION_PREDICTION_MODELS_MOTION_PREDICTION_MODEL_DIFFERENTIAL_DRIVE_H

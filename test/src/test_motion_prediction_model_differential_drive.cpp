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

#include <gtest/gtest.h>

#include <mcl_3dl/motion_prediction_models/motion_prediction_model_differential_drive.h>

static const double EPS = 1.0e-6;

TEST(TestMotionPredictionModelDifferentialDrive, predictWithoutNoise)
{
  const mcl_3dl::Vec3 relative_trans(0.5, -0.1, 0.1);
  const mcl_3dl::Quat relative_quat(mcl_3dl::Vec3(0.1, -0.1, 0.3));

  const mcl_3dl::State6DOF odom_prev(mcl_3dl::Vec3(1.0, 1.1, 1.2), mcl_3dl::Vec3(0.3, 0.2, 0.1));
  const mcl_3dl::State6DOF odom_current(odom_prev.pos_ + odom_prev.rot_ * relative_trans,
                                        odom_prev.rot_ * relative_quat);

  const mcl_3dl::Vec3 state_trans(1.5, 1.2, 1.3);
  const mcl_3dl::Quat state_quat(mcl_3dl::Vec3(-0.3, -0.2, -0.1));
  mcl_3dl::State6DOF state(state_trans, state_quat);
  state.noise_ll_ = 0.0;
  state.noise_la_ = 0.0;
  state.noise_al_ = 0.0;
  state.noise_aa_ = 0.0;

  mcl_3dl::MotionPredictionModelDifferentialDrive predictor(10.0, 10.0);
  predictor.setOdoms(odom_prev, odom_current, 1.0);
  predictor.predict(state);

  const auto expected_state_pos = state_trans + state_quat * relative_trans;
  EXPECT_NEAR(state.pos_.x_, expected_state_pos.x_, EPS);
  EXPECT_NEAR(state.pos_.y_, expected_state_pos.y_, EPS);
  EXPECT_NEAR(state.pos_.z_, expected_state_pos.z_, EPS);

  const mcl_3dl::Quat expected_state_rot = state_quat * relative_quat;
  EXPECT_NEAR(state.rot_.x_, expected_state_rot.x_, EPS);
  EXPECT_NEAR(state.rot_.y_, expected_state_rot.y_, EPS);
  EXPECT_NEAR(state.rot_.z_, expected_state_rot.z_, EPS);
  EXPECT_NEAR(state.rot_.w_, expected_state_rot.w_, EPS);

  // odom_err_integ_lin_ and odom_err_integ_ang_ are zero vectors.
  EXPECT_NEAR(state.odom_err_integ_lin_.x_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.y_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.z_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.x_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.y_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.z_, 0.0, EPS);
}

TEST(TestMotionPredictionModelDifferentialDrive, predictWithoutRotation)
{
  const mcl_3dl::Vec3 relative_trans(-0.5, 0.1, 0.05);
  const float relative_trans_norm = relative_trans.norm();

  const mcl_3dl::State6DOF odom_prev(mcl_3dl::Vec3(5.0, 4.0, 3.0), mcl_3dl::Vec3(0.5, 0.4, 0.3));
  const mcl_3dl::State6DOF odom_current(odom_prev.pos_ + odom_prev.rot_ * relative_trans, odom_prev.rot_);

  const mcl_3dl::Vec3 state_trans(4.0, 3.0, 2.0);
  const float state_yaw_angle = 0.3;
  const mcl_3dl::Quat state_quat(mcl_3dl::Vec3(0.0, 0.0, state_yaw_angle));
  mcl_3dl::State6DOF state(state_trans, state_quat);
  state.noise_ll_ = 0.1;
  state.noise_al_ = 0.2;
  state.noise_la_ = 0.3;
  state.noise_aa_ = 0.4;

  const float err_integ_multiply = 0.9;
  mcl_3dl::MotionPredictionModelDifferentialDrive predictor(10.0, 10.0);
  predictor.setOdoms(odom_prev, odom_current, 1.0);
  predictor.predict(state);

  const auto expected_state_pos = state_trans + state_quat * relative_trans * (1.0 + state.noise_ll_);
  EXPECT_NEAR(state.pos_.x_, expected_state_pos.x_, EPS);
  EXPECT_NEAR(state.pos_.y_, expected_state_pos.y_, EPS);
  EXPECT_NEAR(state.pos_.z_, expected_state_pos.z_, EPS);

  const float yaw_diff = state.noise_la_ * relative_trans_norm;
  const mcl_3dl::Quat expected_state_quat(mcl_3dl::Vec3(0.0, 0.0, state_yaw_angle + yaw_diff));
  EXPECT_NEAR(state.rot_.x_, expected_state_quat.x_, EPS);
  EXPECT_NEAR(state.rot_.y_, expected_state_quat.y_, EPS);
  EXPECT_NEAR(state.rot_.z_, expected_state_quat.z_, EPS);
  EXPECT_NEAR(state.rot_.w_, expected_state_quat.w_, EPS);

  const auto expected_odom_err_integ_lin = (relative_trans * state.noise_ll_) * err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_lin_.x_, expected_odom_err_integ_lin.x_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.y_, expected_odom_err_integ_lin.y_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.z_, expected_odom_err_integ_lin.z_, EPS);

  const float expected_odom_err_integ_acc_z = yaw_diff * err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_ang_.x_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.y_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.z_, expected_odom_err_integ_acc_z, EPS);
}

TEST(TestMotionPredictionModelDifferentialDrive, predictWithoutTranslationAndRollPitch)
{
  const float relative_yaw_angle = 0.2;
  const mcl_3dl::Quat relative_quat(mcl_3dl::Vec3(0.0, 0.0, relative_yaw_angle));

  const mcl_3dl::State6DOF odom_prev(mcl_3dl::Vec3(-5.0, -2.0, -3.0), mcl_3dl::Vec3(0.5, 1.0, 2.0));
  const mcl_3dl::State6DOF odom_current(odom_prev.pos_, odom_prev.rot_ * relative_quat);

  const mcl_3dl::Vec3 state_trans(-5.5, -3.0, -2.0);
  const float state_yaw_angle = 0.3;
  const mcl_3dl::Quat state_quat(mcl_3dl::Vec3(0.0, 0.0, state_yaw_angle));
  mcl_3dl::State6DOF state(state_trans, state_quat);
  state.noise_ll_ = 0.1;
  state.noise_al_ = 0.2;
  state.noise_la_ = 0.3;
  state.noise_aa_ = 0.4;

  const float err_integ_multiply = 1.0 - (1.0 / 10.0);  // single step of the exponential smoother
  mcl_3dl::MotionPredictionModelDifferentialDrive predictor(10.0, 10.0);
  predictor.setOdoms(odom_prev, odom_current, 1.0);
  predictor.predict(state);

  const auto expected_state_pos =
      state_trans + state_quat * mcl_3dl::Vec3(state.noise_al_ * relative_yaw_angle, 0.0, 0.0);
  EXPECT_NEAR(state.pos_.x_, expected_state_pos.x_, EPS);
  EXPECT_NEAR(state.pos_.y_, expected_state_pos.y_, EPS);
  EXPECT_NEAR(state.pos_.z_, expected_state_pos.z_, EPS);

  const float yaw_diff = state.noise_aa_ * relative_yaw_angle;
  const mcl_3dl::Quat expected_state_quat(mcl_3dl::Vec3(0.0, 0.0, relative_yaw_angle + state_yaw_angle + yaw_diff));
  EXPECT_NEAR(state.rot_.x_, expected_state_quat.x_, EPS);
  EXPECT_NEAR(state.rot_.y_, expected_state_quat.y_, EPS);
  EXPECT_NEAR(state.rot_.z_, expected_state_quat.z_, EPS);
  EXPECT_NEAR(state.rot_.w_, expected_state_quat.w_, EPS);

  const auto expected_odom_err_integ_lin =
      mcl_3dl::Vec3(state.noise_al_ * relative_yaw_angle, 0.0, 0.0) * err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_lin_.x_, expected_odom_err_integ_lin.x_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.y_, expected_odom_err_integ_lin.y_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.z_, expected_odom_err_integ_lin.z_, EPS);

  const float expected_odom_err_integ_acc_z = yaw_diff * err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_ang_.x_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.y_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.z_, expected_odom_err_integ_acc_z, EPS);
}

TEST(TestMotionPredictionModelDifferentialDrive, predictWithoutRollPitch)
{
  const mcl_3dl::Vec3 relative_trans(1.0, 0.2, 0.3);
  const float relative_trans_norm = relative_trans.norm();
  const float relative_yaw_angle = 0.2;
  const mcl_3dl::Quat relative_quat(mcl_3dl::Vec3(0.0, 0.0, relative_yaw_angle));

  const mcl_3dl::State6DOF odom_prev(mcl_3dl::Vec3(10.0, -5.0, 0.5), mcl_3dl::Vec3(-2.0, -1.0, -0.5));
  const mcl_3dl::State6DOF odom_current(odom_prev.pos_ + odom_prev.rot_ * relative_trans,
                                        odom_prev.rot_ * relative_quat);

  const mcl_3dl::Vec3 state_trans(11.0, -5.0, 1.0);
  const float state_yaw_angle = 0.3;
  const mcl_3dl::Quat state_quat(mcl_3dl::Vec3(0.0, 0.0, state_yaw_angle));
  mcl_3dl::State6DOF state(state_trans, state_quat);
  state.noise_ll_ = 0.1;
  state.noise_al_ = 0.2;
  state.noise_la_ = 0.3;
  state.noise_aa_ = 0.4;

  const float err_integ_multiply = 1.0 - (1.0 / 10.0);  // single step of the exponential smoother
  mcl_3dl::MotionPredictionModelDifferentialDrive predictor(10.0, 10.0);
  predictor.setOdoms(odom_prev, odom_current, 1.0);
  predictor.predict(state);

  const auto expected_state_pos = state_trans +
                                  state_quat * (relative_trans * (1.0 + state.noise_ll_) +
                                                mcl_3dl::Vec3(state.noise_al_ * relative_yaw_angle, 0.0, 0.0));
  EXPECT_NEAR(state.pos_.x_, expected_state_pos.x_, EPS);
  EXPECT_NEAR(state.pos_.y_, expected_state_pos.y_, EPS);
  EXPECT_NEAR(state.pos_.z_, expected_state_pos.z_, EPS);

  const double yaw_diff = state.noise_la_ * relative_trans_norm + state.noise_aa_ * relative_yaw_angle;
  const mcl_3dl::Quat expected_state_quat(mcl_3dl::Vec3(0.0, 0.0, relative_yaw_angle + state_yaw_angle + yaw_diff));
  EXPECT_NEAR(state.rot_.x_, expected_state_quat.x_, EPS);
  EXPECT_NEAR(state.rot_.y_, expected_state_quat.y_, EPS);
  EXPECT_NEAR(state.rot_.z_, expected_state_quat.z_, EPS);
  EXPECT_NEAR(state.rot_.w_, expected_state_quat.w_, EPS);

  const auto expected_odom_err_integ_lin =
      (relative_trans * state.noise_ll_ + mcl_3dl::Vec3(state.noise_al_ * relative_yaw_angle, 0.0, 0.0)) *
      err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_lin_.x_, expected_odom_err_integ_lin.x_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.y_, expected_odom_err_integ_lin.y_, EPS);
  EXPECT_NEAR(state.odom_err_integ_lin_.z_, expected_odom_err_integ_lin.z_, EPS);

  const float expected_odom_err_integ_acc_z = yaw_diff * err_integ_multiply;
  EXPECT_NEAR(state.odom_err_integ_ang_.x_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.y_, 0.0, EPS);
  EXPECT_NEAR(state.odom_err_integ_ang_.z_, expected_odom_err_integ_acc_z, EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

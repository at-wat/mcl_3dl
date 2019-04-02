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

TEST(TestMotionPredictionModelDifferentialDrive, predict)
{
  mcl_3dl::MotionPredictionModelDifferentialDrive predictor(10.0, 10.0);
  const mcl_3dl::State6DOF odom_2d_prev(mcl_3dl::Vec3(1.0, 0.5, 0.0),
                                        mcl_3dl::Quat(mcl_3dl::Vec3(0.0, 0.0, M_PI / 3.0)));
  const mcl_3dl::Vec3 relative_trans_2d(0.5, 0.0, 0.0);
  const mcl_3dl::Quat relative_quat_2d(mcl_3dl::Vec3(0.0, 0.0, -M_PI / 18.0));
  const mcl_3dl::State6DOF odom_2d_current(odom_2d_prev.pos_ + odom_2d_prev.rot_ * relative_trans_2d,
                                           odom_2d_prev.rot_ * relative_quat_2d);
  predictor.setOdoms(odom_2d_prev, odom_2d_current, 0.1);

  const mcl_3dl::Vec3 relative_trans_s1(0.6, -0.1, 0.0);
  const mcl_3dl::Quat relative_quat_s1(mcl_3dl::Vec3(0.0, 0.0, -M_PI / 17.0));
  mcl_3dl::State6DOF s1(odom_2d_prev.pos_ + odom_2d_prev.rot_ * relative_trans_s1,
                        odom_2d_prev.rot_ * relative_quat_s1);
  s1.noise_ll_ = 0.1;
  s1.noise_la_ = 0.05;
  s1.noise_al_ = 0.05;
  s1.noise_aa_ = 0.05;
  predictor.predict(s1);
  EXPECT_NEAR(s1.pos_.x_, 1.7501203, 1.0e-6);
  EXPECT_NEAR(s1.pos_.y_, 1.3939149, 1.0e-6);
  EXPECT_NEAR(s1.pos_.z_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.rot_.x_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.rot_.y_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.rot_.z_, 0.3530190, 1.0e-6);
  EXPECT_NEAR(s1.rot_.w_, 0.9356161, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_lin_.x_, 0.0581395, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_lin_.y_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_lin_.z_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_ang_.x_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_ang_.y_, 0.0, 1.0e-6);
  EXPECT_NEAR(s1.odom_err_integ_ang_.z_, 0.0333894, 1.0e-6);

  const mcl_3dl::State6DOF odom_3d_prev(mcl_3dl::Vec3(1.0, 0.5, 0.4),
                                        mcl_3dl::Quat(mcl_3dl::Vec3(M_PI / 12.0, -M_PI / 10.0, M_PI / 3.0)));
  const mcl_3dl::Vec3 relative_trans_3d(0.4, 0.0, 0.0);
  const mcl_3dl::Quat relative_quat_3d(mcl_3dl::Vec3(M_PI / 36.0, M_PI / 12.0, -M_PI / 18.0));
  const mcl_3dl::State6DOF odom_3d_current(odom_3d_prev.pos_ + odom_3d_prev.rot_ * relative_trans_3d,
                                           odom_3d_prev.rot_ * relative_quat_3d);
  predictor.setOdoms(odom_3d_prev, odom_3d_current, 0.1);

  const mcl_3dl::Vec3 relative_trans_s2(0.5, 0.05, 0.1);
  const mcl_3dl::Quat relative_quat_s2(mcl_3dl::Vec3(M_PI / 30.0, M_PI / 16.0, -M_PI / 17.0));
  mcl_3dl::State6DOF s2(odom_2d_prev.pos_ + odom_2d_prev.rot_ * relative_trans_s2,
                        odom_2d_prev.rot_ * relative_quat_s2);
  s2.noise_ll_ = 0.1;
  s2.noise_la_ = 0.05;
  s2.noise_al_ = 0.05;
  s2.noise_aa_ = 0.05;
  predictor.predict(s2);
  EXPECT_NEAR(s2.pos_.x_, 1.4980695, 1.0e-6);
  EXPECT_NEAR(s2.pos_.y_, 1.2981024, 1.0e-6);
  EXPECT_NEAR(s2.pos_.z_, 0.0109197, 1.0e-6);
  EXPECT_NEAR(s2.rot_.x_, -0.0109055, 1.0e-6);
  EXPECT_NEAR(s2.rot_.y_, 0.2461149, 1.0e-6);
  EXPECT_NEAR(s2.rot_.z_, 0.3333000, 1.0e-6);
  EXPECT_NEAR(s2.rot_.w_, 0.9100657, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_lin_.x_, 0.0560445, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_lin_.y_, 0.0, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_lin_.z_, 0.0, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_ang_.x_, 0.0, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_ang_.y_, 0.0, 1.0e-6);
  EXPECT_NEAR(s2.odom_err_integ_ang_.z_, 0.0362446, 1.0e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

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

#include <mcl_3dl/imu_measurement_models/imu_measurement_model_gravity.h>

TEST(ImuMeasurementModelGravity, measure)
{
  mcl_3dl::ImuMeasurementModelGravity measurement(M_PI / 4.0);
  measurement.setAccMeasure(mcl_3dl::Vec3(0.1, 0.2, 0.3));
  mcl_3dl::State6DOF s1(mcl_3dl::Vec3(0.0, 0.0, 0.0),
                        mcl_3dl::Quat(mcl_3dl::Vec3(M_PI / 18.0, M_PI / 6.0, M_PI / -2.0)));
  EXPECT_NEAR(measurement.measure(s1), 0.2678633, 1.0e-6);

  measurement.setAccMeasure(mcl_3dl::Vec3(-0.3, -0.2, -0.1));
  mcl_3dl::State6DOF s2(mcl_3dl::Vec3(0.0, 0.0, 0.0),
                        mcl_3dl::Quat(mcl_3dl::Vec3(-M_PI / 12.0, M_PI / 36.0, M_PI * 1.5)));
  EXPECT_NEAR(measurement.measure(s2), 0.0604828, 1.0e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

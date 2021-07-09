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

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <mcl_3dl/point_cloud_random_samplers/point_cloud_uniform_sampler.h>

#include <gtest/gtest.h>

TEST(PointCloudUniformSampler, Sampling)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input(new pcl::PointCloud<pcl::PointXYZ>);
  pc_input->width = 1;
  pc_input->height = 0;
  pc_input->header.frame_id = "frame0";
  pc_input->header.stamp = 12345;

  const float points_ref[][3] =
      {
          {10, 11, 12},
          {20, 21, 22},
          {30, 31, 32},
      };
  for (const auto& p_ref : points_ref)
  {
    pc_input->push_back(pcl::PointXYZ(p_ref[0], p_ref[1], p_ref[2]));
  }

  mcl_3dl::PointCloudUniformSampler<pcl::PointXYZ> sampler;

  for (size_t num = 1; num < 4; num++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output = sampler.sample(pc_input, num);

    // Check header and number of the points
    ASSERT_EQ(pc_output->header.frame_id, pc_input->header.frame_id);
    ASSERT_EQ(pc_output->header.stamp, pc_input->header.stamp);
    ASSERT_EQ(pc_output->height, 1u);
    ASSERT_EQ(pc_output->width, num);

    // Check that the all sampled points are in the original point array
    for (const pcl::PointXYZ& p : *pc_output)
    {
      bool found = false;
      for (const auto& p_ref : points_ref)
      {
        if (p_ref[0] == p.x && p_ref[1] == p.y && p_ref[2] == p.z)
        {
          found = true;
          break;
        }
      }
      ASSERT_TRUE(found) << "A sampled point is not in the original points array";
    }
  }

  // Make sure that the sampler returns 0 point output for 0 point input
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output0 = sampler.sample(pc_input, 0);
  ASSERT_EQ(pc_output0->points.size(), 0u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

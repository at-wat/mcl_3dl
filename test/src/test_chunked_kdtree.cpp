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

#include <cstddef>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <mcl_3dl/chunked_kdtree.h>

TEST(ChunkedKdtree, RadiusSearch)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  pc.push_back(pcl::PointXYZ(0.5, 0.5, 0.5));   // 0
  pc.push_back(pcl::PointXYZ(0.8, 0.0, 0.0));   // 1
  pc.push_back(pcl::PointXYZ(1.3, 0.0, 0.0));   // 2
  pc.push_back(pcl::PointXYZ(0.0, 0.2, 0.0));   // 3
  pc.push_back(pcl::PointXYZ(0.0, -0.3, 0.0));  // 4

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ> kdtree(1.0, 0.3);
  kdtree.setInputCloud(pc.makeShared());

  std::vector<int> id;
  std::vector<float> dist;

  kdtree.radiusSearch(
      pcl::PointXYZ(0.5, 0.5, 0.5),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 0);

  kdtree.radiusSearch(
      pcl::PointXYZ(0.5, 0.4, 0.5),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 0);

  kdtree.radiusSearch(
      pcl::PointXYZ(1.05, 0.0, 0.0),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 1);

  kdtree.radiusSearch(
      pcl::PointXYZ(1.1, 0.0, 0.0),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 2);

  kdtree.radiusSearch(
      pcl::PointXYZ(0.0, -0.05, 0.0),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 3);

  kdtree.radiusSearch(
      pcl::PointXYZ(0.0, -0.15, 0.0),
      0.3, id, dist, 1);
  ASSERT_EQ(id.size(), 1u);
  ASSERT_EQ(id[0], 4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

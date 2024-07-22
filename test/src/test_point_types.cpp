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

#include <gtest/gtest.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <mcl_3dl/point_types.h>

TEST(PointTypes, VoxelGrid)
{
  pcl::PointCloud<mcl_3dl::PointXYZIL>::Ptr pc(new pcl::PointCloud<mcl_3dl::PointXYZIL>);
  pc->resize(3);
  pc->points[0].x = 1.0;
  pc->points[0].y = 2.0;
  pc->points[0].z = 3.0;
  pc->points[0].intensity = 4.0;
  pc->points[0].label = 1;
  pc->points[1].x = 1.02;
  pc->points[1].y = 2.02;
  pc->points[1].z = 3.02;
  pc->points[1].intensity = 5.0;
  pc->points[1].label = 3;
  pc->points[2].x = -1.0;
  pc->points[2].y = -2.0;
  pc->points[2].z = -3.0;
  pc->points[2].intensity = 6.0;
  pc->points[2].label = 4;

  pcl::PointCloud<mcl_3dl::PointXYZIL>::Ptr pc2(new pcl::PointCloud<mcl_3dl::PointXYZIL>);
  pcl::VoxelGrid<mcl_3dl::PointXYZIL> ds;
  ds.setInputCloud(pc);
  ds.setLeafSize(0.1, 0.1, 0.1);
  ds.filter(*pc2);

  ASSERT_EQ(pc2->size(), 2u);
  int num_1(0), num_4(0);
  for (auto& p : *pc2)
  {
    switch (p.label)
    {
      case 1u:
      case 3u:
        ASSERT_FLOAT_EQ(p.x, 1.01);
        ASSERT_FLOAT_EQ(p.y, 2.01);
        ASSERT_FLOAT_EQ(p.z, 3.01);
        ASSERT_FLOAT_EQ(p.intensity, 4.5);
        num_1++;
        break;
      case 4u:
        ASSERT_EQ(p.x, -1.0);
        ASSERT_EQ(p.y, -2.0);
        ASSERT_EQ(p.z, -3.0);
        ASSERT_EQ(p.intensity, 6.0);
        num_4++;
        break;
      default:
        ASSERT_TRUE(false)
            << "Unexpected point label (" << p.label << "); "
            << "original labels are [1, 3, 4]";
        break;
    }
  }
  ASSERT_TRUE(num_1 == 1 && num_4 == 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

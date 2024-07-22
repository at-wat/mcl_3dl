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

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/raycasts/raycast_using_dda.h>
#include <mcl_3dl/raycasts/raycast_using_kdtree.h>

TEST(Raycast, Collision)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -1.0; y < 1.0; y += 0.1)
  {
    for (float z = -1.0; z < 1.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(0.5, y, z));
    }
  }
  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());
  const float hit_range = 0.1 * std::sqrt(3.f);
  mcl_3dl::RaycastUsingKDTree<pcl::PointXYZ> raycaster(0.1, 0.1, 0.1, hit_range);
  for (float y = -0.8; y < 0.8; y += 0.11)
  {
    for (float z = -0.8; z < 0.8; z += 0.13)
    {
      bool collision = false;
      raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, y * 2.0, z * 2.0));
      mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
      while (raycaster.getNextCastResult(point))
      {
        if (point.collision_)
        {
          collision = true;
          EXPECT_NEAR((point.pos_ - mcl_3dl::Vec3(0.5, y, z)).norm(), 0.0, 0.2);
          break;
        }
      }
      ASSERT_TRUE(collision);
    }
  }
  for (float y = -1.0; y < 1.0; y += 0.11)
  {
    for (float z = -1.0; z < 1.0; z += 0.13)
    {
      raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5 - hit_range, y, z));
      mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
      bool collision = false;
      while (raycaster.getNextCastResult(point))
      {
        if (point.collision_)
          collision = true;
      }
      ASSERT_FALSE(collision);
    }
  }
  {
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5, 3.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    bool collision = false;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
        collision = true;
    }
    ASSERT_FALSE(collision);
  }
}

TEST(Raycast, CollisionTolerance)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -1.0; y < 1.0; y += 0.05)
  {
    for (float z = -1.0; z < 1.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(0.5, y, z));
    }
  }

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());

  {
    mcl_3dl::RaycastUsingKDTree<pcl::PointXYZ> raycaster(0.05, 0.1, 0.1, 0.1 * std::sqrt(3.f));
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5, 0.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    bool collision = false;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
      {
        collision = true;
        break;
      }
    }
    ASSERT_TRUE(collision);
  }
  {
    const float hit_range = 0.15 * std::sqrt(3.f);
    const float epsilon = 0.01;
    mcl_3dl::RaycastUsingKDTree<pcl::PointXYZ> raycaster(0.1, 0.15, 0.15, hit_range);
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5 - hit_range - epsilon, 0.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    bool collision = false;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
      {
        collision = true;
        break;
      }
    }
    ASSERT_FALSE(collision);
  }
}

TEST(Raycast, SinAng)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -1.0; y < 1.0; y += 0.1)
  {
    for (float z = -1.0; z < 1.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(0.5, y, z));
    }
  }
  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());
  mcl_3dl::RaycastUsingKDTree<pcl::PointXYZ> raycaster(0.1, 0.1, 0.1, 0.1 * std::sqrt(3.f));

  {
    bool collision = false;
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, 0.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
      {
        EXPECT_NEAR(point.sin_angle_, 1.0, 0.1);
        collision = true;
        break;
      }
    }
    ASSERT_TRUE(collision);
  }
  {
    bool collision = false;
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 5.0, 0.0), mcl_3dl::Vec3(1.0, -5.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
      {
        EXPECT_NEAR(point.sin_angle_, sinf(0.5 / 5.0), 0.05);
        collision = true;
        break;
      }
    }
    ASSERT_TRUE(collision);
  }
  {
    bool collision = false;
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 3.0, 0.0), mcl_3dl::Vec3(1.0, -3.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
      {
        EXPECT_NEAR(point.sin_angle_, sinf(0.5 / 3.0), 0.05);
        collision = true;
        break;
      }
    }
    ASSERT_TRUE(collision);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

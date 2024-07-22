/*
 * Copyright (c) 2020, the mcl_3dl authors
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
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/raycasts/raycast_using_dda.h>

TEST(RaycastUsingDDA, Collision)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -1.0; y < 1.0; y += 0.1)
  {
    for (float z = -1.0; z < 1.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(0.5, y, z));
    }
  }
  // Add dummy points to get edges of map
  pc.push_back(pcl::PointXYZ(-2.05, -2.05, -2.05));
  pc.push_back(pcl::PointXYZ(2.05, 2.05, 2.05));

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());
  const float epsilon = 0.05;
  const float hit_range = std::sqrt(3.0) * 0.1;
  mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> raycaster(0.1, 0.1, 0.1, 0.1, 0.5, hit_range);
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
      bool collision = false;
      raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5 - hit_range - epsilon, y, z));
      mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
      while (raycaster.getNextCastResult(point))
      {
        if (point.collision_)
          collision = true;
      }
      ASSERT_FALSE(collision);
    }
  }
  {
    bool collision = false;
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5, 3.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
    while (raycaster.getNextCastResult(point))
    {
      if (point.collision_)
        collision = true;
    }
    ASSERT_FALSE(collision);
  }
}

TEST(RaycastUsingDDA, CollisionTolerance)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -1.0; y < 1.0; y += 0.05)
  {
    for (float z = -1.0; z < 1.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(0.5, y, z));
    }
  }
  // Add dummy points to get edges of map
  pc.push_back(pcl::PointXYZ(-2.05, -2.05, -2.05));
  pc.push_back(pcl::PointXYZ(2.05, 2.05, 2.05));

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());

  {
    mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> raycaster(0.05, 0.1, 0.1, 0.1, 0.5, sqrt(3.0) * 0.1);

    bool collision = false;
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5, 0.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
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
    bool collision = false;
    const float hit_range = std::sqrt(3.0) * 0.15;
    mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> raycaster(0.1, 0.15, 0.15, 0.15, 0.5, hit_range);
    raycaster.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(0.5 - hit_range, 0.0, 0.0));
    mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
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

void compareRayWaypoints(const char* name, typename mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree,
                         mcl_3dl::RaycastUsingDDA<pcl::PointXYZ>& raycaster, const mcl_3dl::Vec3& ray_begin,
                         const mcl_3dl::Vec3& ray_end, const std::vector<mcl_3dl::Vec3> expected_points,
                         const bool expected_collision)
{
  raycaster.setRay(kdtree, ray_begin, ray_end);
  std::vector<mcl_3dl::Vec3> actual_points;
  mcl_3dl::Raycast<pcl::PointXYZ>::CastResult pos;
  bool collision = false;
  while (raycaster.getNextCastResult(pos))
  {
    actual_points.push_back(pos.pos_);
    if (pos.collision_)
    {
      collision = true;
      break;
    }
  }
  EXPECT_EQ(expected_collision, collision) << "case: " << name;
  ASSERT_EQ(expected_points.size(), actual_points.size()) << "case: " << name;
  for (size_t i = 0; i < expected_points.size(); ++i)
  {
    EXPECT_NEAR(expected_points[i].x_, actual_points[i].x_, 1.0e-6f) << "case: " << name;
    EXPECT_NEAR(expected_points[i].y_, actual_points[i].y_, 1.0e-6f) << "case: " << name;
    EXPECT_NEAR(expected_points[i].z_, actual_points[i].z_, 1.0e-6f) << "case: " << name;
  }
}

TEST(RaycastUsingDDA, Waypoints)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  pc.push_back(pcl::PointXYZ(0.9, 0.6, 0.0));
  // Add dummy points for getting map edges
  pc.push_back(pcl::PointXYZ(-2.05, -2.05, -2.05));
  pc.push_back(pcl::PointXYZ(2.05, 2.05, 2.05));

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());
  mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> raycaster(0.1, 0.1, 0.1, 0.1, 0.5, 0.0);

  {
    const std::vector<mcl_3dl::Vec3> expected_points =
        {
            mcl_3dl::Vec3(0.1, 0.0, 0.0),
            mcl_3dl::Vec3(0.1, 0.1, 0.0),
            mcl_3dl::Vec3(0.2, 0.1, 0.0),
            mcl_3dl::Vec3(0.2, 0.2, 0.0),
            mcl_3dl::Vec3(0.3, 0.2, 0.0),
            mcl_3dl::Vec3(0.4, 0.2, 0.0),
            mcl_3dl::Vec3(0.4, 0.3, 0.0),
            mcl_3dl::Vec3(0.5, 0.3, 0.0),
            mcl_3dl::Vec3(0.5, 0.4, 0.0),
            mcl_3dl::Vec3(0.6, 0.4, 0.0),
            mcl_3dl::Vec3(0.7, 0.4, 0.0),
            mcl_3dl::Vec3(0.7, 0.5, 0.0),
            mcl_3dl::Vec3(0.8, 0.5, 0.0),
            mcl_3dl::Vec3(0.8, 0.6, 0.0),
            mcl_3dl::Vec3(0.9, 0.6, 0.0),
        };
    compareRayWaypoints("Waypoints#1", kdtree, raycaster,
                        mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.2, 0.8, 0.0),
                        expected_points, true);
  }
  {
    // Start and end grids are same, but waypoints are different.
    const std::vector<mcl_3dl::Vec3> expected_points =
        {
            mcl_3dl::Vec3(0.0, 0.1, 0.0),
            mcl_3dl::Vec3(0.1, 0.1, 0.0),
            mcl_3dl::Vec3(0.1, 0.2, 0.0),
            mcl_3dl::Vec3(0.2, 0.2, 0.0),
            mcl_3dl::Vec3(0.3, 0.2, 0.0),
            mcl_3dl::Vec3(0.3, 0.3, 0.0),
            mcl_3dl::Vec3(0.4, 0.3, 0.0),
            mcl_3dl::Vec3(0.4, 0.4, 0.0),
            mcl_3dl::Vec3(0.5, 0.4, 0.0),
            mcl_3dl::Vec3(0.6, 0.4, 0.0),
            mcl_3dl::Vec3(0.6, 0.5, 0.0),
            mcl_3dl::Vec3(0.7, 0.5, 0.0),
            mcl_3dl::Vec3(0.7, 0.6, 0.0),
            mcl_3dl::Vec3(0.8, 0.6, 0.0),
            mcl_3dl::Vec3(0.9, 0.6, 0.0),
        };
    compareRayWaypoints("Waypoints#2", kdtree, raycaster,
                        mcl_3dl::Vec3(-0.04, 0.04, 0.0), mcl_3dl::Vec3(1.16, 0.84, 0.0),
                        expected_points, true);
  }
}

TEST(RaycastUsingDDA, Intersection)
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  pc.push_back(pcl::PointXYZ(0.6, -0.4, 0.0));
  // Add dummy points for getting map edges
  pc.push_back(pcl::PointXYZ(-2.1, -2.1, -2.1));
  pc.push_back(pcl::PointXYZ(2.1, 2.1, 2.1));

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 1.0));
  kdtree->setInputCloud(pc.makeShared());
  mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> raycaster(0.05, 0.05, 0.05, 0.2, 0.01, 0.0);
  {
    const std::vector<mcl_3dl::Vec3> expected_points =
        {
            mcl_3dl::Vec3(0.2, 0.0, 0.0),
            mcl_3dl::Vec3(0.2, -0.2, 0.0),
            mcl_3dl::Vec3(0.4, -0.2, 0.0),
            mcl_3dl::Vec3(0.6, -0.2, 0.0),
            mcl_3dl::Vec3(0.6, -0.4, 0.0),
        };
    compareRayWaypoints("Intersection#1", kdtree, raycaster,
                        mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, -0.55, 0.0),
                        expected_points, true);
  }
  {
    const std::vector<mcl_3dl::Vec3> expected_points =
        {
            mcl_3dl::Vec3(0.2, 0.0, 0.0),
            mcl_3dl::Vec3(0.2, -0.2, 0.0),
            mcl_3dl::Vec3(0.4, -0.2, 0.0),
            mcl_3dl::Vec3(0.6, -0.2, 0.0),
            mcl_3dl::Vec3(0.6, -0.4, 0.0),  // The ray passes through this voxel
            mcl_3dl::Vec3(0.8, -0.4, 0.0),
            mcl_3dl::Vec3(1.0, -0.4, 0.0),
        };
    // The ray does not hit the obstacle as the distance between them is larger than the threshold.
    compareRayWaypoints("Intersection#2", kdtree, raycaster,
                        mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.1, -0.55, 0.0),
                        expected_points, false);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

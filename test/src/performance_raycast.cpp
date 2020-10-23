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

#include <boost/chrono.hpp>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/raycasts/raycast_using_dda.h>
#include <mcl_3dl/raycasts/raycast_using_kdtree.h>

void performanceTestRaycast(const float chunk_size)
{
  std::cerr << "## Chunk size: " << chunk_size << std::endl;
  const auto ts = boost::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -50.0; y < 50.0; y += 0.1)
  {
    for (float z = -50.0; z < 50.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(5.0, y, z));
      pc.push_back(pcl::PointXYZ(y + 10.0, 1.5, z));
    }
  }
  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(chunk_size, 0.1));
  kdtree->setInputCloud(pc.makeShared());
  const auto tnow = boost::chrono::high_resolution_clock::now();
  std::cerr << "- Generate kdtree: " << boost::chrono::duration<float>(tnow - ts).count() << " sec" << std::endl;

  const auto ts2 = boost::chrono::high_resolution_clock::now();
  size_t collision_cnt = 0;
  size_t cnt = 0;
  mcl_3dl::RaycastUsingKDTree<pcl::PointXYZ> ray(0.1, 0.1, 0.1, 0.1 * std::sqrt(3.f));
  const auto ts3 = boost::chrono::high_resolution_clock::now();
  for (float y = -50.0; y < 50.0; y += 1.2)
  {
    for (float z = -50.0; z < 50.0; z += 1.1)
    {
      cnt++;
      ray.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, y * 2.0, z * 2.0));
      mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
      while (ray.getNextCastResult(point))
      {
        if (point.collision_)
        {
          collision_cnt++;
          break;
        }
      }
    }
  }
  std::cerr << "- Collisions: " << collision_cnt << "/" << cnt << std::endl;
  const auto tnow2 = boost::chrono::high_resolution_clock::now();
  std::cerr << "- mcl_3dl::RaycastUsingKDTree Init: " << boost::chrono::duration<float>(ts3 - ts2).count() << " sec"
            << std::endl;
  std::cerr << "- mcl_3dl::RaycastUsingKDTree: " << boost::chrono::duration<float>(tnow2 - ts3).count() << " sec"
            << std::endl;
  std::cerr << std::endl;
}

void performanceTestRaycastUsingDDA(const float dda_grid_size)
{
  std::cerr << "## DDA grid size: " << dda_grid_size << std::endl;
  const auto ts = boost::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float y = -50.0; y < 50.0; y += 0.1)
  {
    for (float z = -50.0; z < 50.0; z += 0.1)
    {
      pc.push_back(pcl::PointXYZ(5.0, y, z));
      pc.push_back(pcl::PointXYZ(y + 10.0, 1.5, z));
    }
  }
  // Add dummy points to get edges of map
  pc.push_back(pcl::PointXYZ(-1.05, -50.05, -50.05));
  pc.push_back(pcl::PointXYZ(2.05, 50.05, 50.05));

  mcl_3dl::ChunkedKdtree<pcl::PointXYZ>::Ptr kdtree(new mcl_3dl::ChunkedKdtree<pcl::PointXYZ>(10.0, 0.1));
  kdtree->setInputCloud(pc.makeShared());
  const auto tnow = boost::chrono::high_resolution_clock::now();
  std::cerr << "- Generate kdtree: " << boost::chrono::duration<float>(tnow - ts).count() << " sec" << std::endl;

  const auto ts2 = boost::chrono::high_resolution_clock::now();
  mcl_3dl::RaycastUsingDDA<pcl::PointXYZ> ray(0.1, 0.1, 0.1, dda_grid_size, 0.5, dda_grid_size * std::sqrt(3));
  // Set dummy ray to initialize DDA.
  ray.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, 0.0, 0.0));
  size_t collision_cnt = 0;
  size_t cnt = 0;
  const auto ts3 = boost::chrono::high_resolution_clock::now();
  for (float y = -50.0; y < 50.0; y += 1.2)
  {
    for (float z = -50.0; z < 50.0; z += 1.1)
    {
      cnt++;
      ray.setRay(kdtree, mcl_3dl::Vec3(0.0, 0.0, 0.0), mcl_3dl::Vec3(1.0, y * 2.0, z * 2.0));
      mcl_3dl::Raycast<pcl::PointXYZ>::CastResult point;
      while (ray.getNextCastResult(point))
      {
        if (point.collision_)
        {
          collision_cnt++;
          break;
        }
      }
    }
  }
  std::cerr << "- Collisions: " << collision_cnt << "/" << cnt << std::endl;
  const auto tnow2 = boost::chrono::high_resolution_clock::now();
  std::cerr << "- mcl_3dl::RaycastUsingDDA Init: " << boost::chrono::duration<float>(ts3 - ts2).count() << " sec"
            << std::endl;
  std::cerr << "- mcl_3dl::RaycastUsingDDA: " << boost::chrono::duration<float>(tnow2 - ts3).count() << " sec"
            << std::endl;
  std::cerr << std::endl;
}

int main(int argc, char** argv)
{
  performanceTestRaycast(1.0);
  performanceTestRaycast(2.0);
  performanceTestRaycast(5.0);
  performanceTestRaycast(10.0);
  performanceTestRaycast(20.0);
  performanceTestRaycast(50.0);
  performanceTestRaycast(100.0);

  performanceTestRaycastUsingDDA(0.1);
  performanceTestRaycastUsingDDA(0.2);
  performanceTestRaycastUsingDDA(0.3);
  performanceTestRaycastUsingDDA(0.4);
  performanceTestRaycastUsingDDA(0.5);

  return 0;
}

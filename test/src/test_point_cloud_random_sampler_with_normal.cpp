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

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include <pcl/point_types.h>

#include <mcl_3dl/point_cloud_random_samplers/point_cloud_sampler_with_normal.h>
#include <mcl_3dl/point_types.h>

namespace mcl_3dl
{
namespace test
{
std::vector<State6DOF> buildPoseCovarianceMatrix(const double yaw,
                                                 const double front_std_dev,
                                                 const double side_std_dev)
{
  Eigen::Matrix2d vt;
  vt(0, 0) = std::cos(yaw);
  vt(0, 1) = -std::sin(yaw);
  vt(1, 0) = std::sin(yaw);
  vt(1, 1) = std::cos(yaw);
  Eigen::Matrix2d m;
  m(0, 0) = std::pow(front_std_dev, 2);
  m(0, 1) = 0;
  m(1, 0) = 0;
  m(1, 1) = std::pow(side_std_dev, 2);
  const Eigen::Matrix2d xv_cov = vt.transpose() * m * vt;

  std::vector<State6DOF> result(6);
  for (auto& state : result)
  {
    for (size_t i = 0; i < 6; ++i)
    {
      state[i] = 0.0;
    }
  }
  result[0][0] = xv_cov(0, 0);
  result[1][0] = xv_cov(1, 0);
  result[0][1] = xv_cov(0, 1);
  result[1][1] = xv_cov(1, 1);
  return result;
}

void buildWall(pcl::PointCloud<PointXYZIL>::Ptr result_points, double rotation_angle, int wall_length)
{
  pcl::PointCloud<PointXYZIL> points;
  for (int ny = 0; ny < wall_length; ++ny)
  {
    for (int nz = 0; nz < wall_length; ++nz)
    {
      PointXYZIL point;
      point.x = 20.0;
      point.y = (ny - wall_length / 2) * 0.05;
      point.z = (nz - wall_length / 2) * 0.05;
      point.label = 0;
      point.intensity = 3000;
      points.push_back(point);
    }
  }
  const State6DOF s(mcl_3dl::Vec3(0, 0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), rotation_angle));
  s.transform(points);
  result_points->insert(result_points->end(), points.begin(), points.end());
}

TEST(PointCloudSamplerWithNormal, Sampling)
{
  pcl::PointCloud<PointXYZIL>::Ptr pc(new pcl::PointCloud<PointXYZIL>());
  int wall_length = 20;
  // Wall at right angles to the first principal component of particles
  buildWall(pc, 0, wall_length);
  // Wall parallel to the first principal component of particles
  buildWall(pc, M_PI / 2, wall_length);

  const double robot_yaw = M_PI / 6;
  const State6DOF mean(mcl_3dl::Vec3(3.5, -5.0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), robot_yaw));
  const std::vector<State6DOF> cov_matrix = buildPoseCovarianceMatrix(robot_yaw, 1.0, 0.2);

  // Fix random seeds to avoid flaky results
  const std::vector<unsigned int> seeds =
      {
          12345,
          23456,
          34567,
          45678,
          56789,
      };

  struct ParameterSet
  {
    double perform_weighting_ratio;
    double max_weight_ratio;
    double max_weight;
    double expected_ratio_min;
    double expected_ratio_max;
  };
  const std::vector<ParameterSet> parameters =
      {
          // Weights of points in the wall at right angles: 10, weights of points in the parallel wall: 1
          {2.0, 4.0, 10.0, 0.85, 1.0},
          // Weights of points in the wall at right angles: 1, weights of points in the parallel wall: 1
          {6.0, 7.0, 10.0, 0.4, 0.6},
          // Weights of points in the wall at right angles: 3, weights of points in the parallel wall: 1
          {2.0, 8.0, 5.0, 0.65, 0.85},
      };

  for (const unsigned int seed : seeds)
  {
    PointCloudSamplerWithNormal<PointXYZIL> sampler(seed);
    sampler.setParticleStatistics(mean, cov_matrix);
    const int sample_num = 100;
    for (const ParameterSet& parameter : parameters)
    {
      sampler.setParameters(parameter.perform_weighting_ratio, parameter.max_weight_ratio, parameter.max_weight, 0.4);
      const pcl::PointCloud<PointXYZIL>::Ptr extracted_cloud = sampler.sample(pc, sample_num);
      EXPECT_EQ(sample_num, static_cast<int>(extracted_cloud->size()));
      // count[0] : numbers of points chosen from the wall at right angles
      // count[1] : numbers of points chosen from the parallel wall
      std::vector<int> counts(2, 0);
      for (const auto& extracted_point : *extracted_cloud)
      {
        const auto pred = [&extracted_point](const PointXYZIL& p)
        {
          return (p.x == extracted_point.x) && (p.y == extracted_point.y) && (p.z == extracted_point.z);
        };
        const size_t index = std::find_if(pc->begin(), pc->end(), pred) - pc->begin();
        ++counts[index / std::pow(wall_length, 2)];
      }
      const double ratio = static_cast<double>(counts[0]) / (counts[0] + counts[1]);
      EXPECT_GE(ratio, parameter.expected_ratio_min) << "Failed at seed #" << seed;
      EXPECT_LE(ratio, parameter.expected_ratio_max) << "Failed at seed #" << seed;
    }
  }

  PointCloudSamplerWithNormal<PointXYZIL> sampler;
  sampler.setParticleStatistics(mean, cov_matrix);
  pcl::PointCloud<PointXYZIL>::Ptr invalid_cloud(new pcl::PointCloud<PointXYZIL>());
  // Empty cloud
  EXPECT_EQ(0u, sampler.sample(invalid_cloud, 100)->size());
  // Point cloud size is smaller than requested sample number
  invalid_cloud->push_back(PointXYZIL());
  EXPECT_EQ(1u, sampler.sample(invalid_cloud, 100)->size());
}
}  // namespace test
}  // namespace mcl_3dl

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

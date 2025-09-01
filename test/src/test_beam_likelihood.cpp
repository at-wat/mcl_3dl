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
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
template <class POINT_TYPE>
class DummySampler : public PointCloudRandomSampler<POINT_TYPE>
{
public:
  DummySampler()
  {
  }

  typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr& pc,
      const size_t num) const final
  {
    typename pcl::PointCloud<POINT_TYPE>::Ptr output(
        new pcl::PointCloud<POINT_TYPE>);
    output->header = pc->header;

    if (pc->points.size() == 0)
    {
      return output;
    }

    output->points.reserve(pc->points.size());
    for (size_t i = 0; i < pc->points.size(); i++)
    {
      output->push_back(pc->points[i]);
    }

    return output;
  }
};
}  // namespace mcl_3dl

TEST(BeamModel, LikelihoodFunc)
{
  pcl::PointCloud<mcl_3dl::LidarMeasurementModelBase::PointType> raw_pc;
  for (float y = -0.2; y <= 0.2; y += 0.1)
  {
    for (float z = -0.2; z <= 0.2; z += 0.1)
    {
      mcl_3dl::LidarMeasurementModelBase::PointType p;
      p.x = 2;
      p.y = y;
      p.z = z;
      raw_pc.push_back(p);
    }
  }
  // Add points that should get filtered out
  mcl_3dl::LidarMeasurementModelBase::PointType p;
  p.z = 5;
  raw_pc.push_back(p);
  p.z = -5;
  raw_pc.push_back(p);

  pcl::PointCloud<mcl_3dl::LidarMeasurementModelBase::PointType> pc_map = raw_pc;
  {
    // Add dummy points to expand DDA grid size.
    mcl_3dl::LidarMeasurementModelBase::PointType p;
    p.x = -100.05;
    p.y = 100;
    p.z = -0.05;
    pc_map.push_back(p);
    p.x = 100;
    p.y = -100.05;
    p.z = 4;
    pc_map.push_back(p);
  }

  mcl_3dl::ChunkedKdtree<mcl_3dl::LidarMeasurementModelBase::PointType>::Ptr kdtree(
      new mcl_3dl::ChunkedKdtree<mcl_3dl::LidarMeasurementModelBase::PointType>(10.0, 1.0));
  kdtree->setInputCloud(pc_map.makeShared());

  // A dummy point sampler that returns all the points from the input cloud
  const mcl_3dl::DummySampler<mcl_3dl::LidarMeasurementModelBase::PointType> sampler;
  for (int method = 0; method < 2; ++method)
  {
    for (int mode = 0; mode < 2; ++mode)
    {
      for (double hr = 0.0; hr <= 1.0; hr += 0.2)
      {
        auto params = std::make_shared<mcl_3dl::LidarMeasurementModelBeamParameters>();
        params->map_grid_x_ = 0.1;
        params->map_grid_y_ = 0.1;
        params->map_grid_z_ = 0.1;
        params->num_points_default_ = static_cast<int>(raw_pc.size());
        params->beam_likelihood_min_ = 0.2;
        params->hit_range_ = hr;
        params->use_raycast_using_dda_ = method == 1;
        params->add_penalty_short_only_mode_ = mode == 1;
        params->dda_grid_size_ = 0.1;
        params->clip_z_min_ = -0.3;
        params->clip_z_max_ = 4.1;

        mcl_3dl::LidarMeasurementModelBeam model(params);
        const auto pc = model.filter(raw_pc.makeShared(), sampler);
        ASSERT_EQ(pc->points.size(), raw_pc.points.size() - 2);
        for (const auto& p : pc->points)
        {
          ASSERT_LT(p.z, 4.1);
          ASSERT_GT(p.z, -0.3);
        }

        std::cerr << "use_raycast_using_dda: " << (method == 1) << ", ";
        std::cerr << "add_penalty_short_only_mode: " << (mode == 1) << ", ";
        std::cerr << "hit_range: " << hr << std::endl;
        for (int i = -50; i < 50; i++)
        {
          if (i == 0)
            std::cerr << "0";
          else if (i % 10 == 0)
            std::cerr << "|";
          else
            std::cerr << " ";
        }
        std::cerr << std::endl;
        for (int i = -50; i < 50; i++)
        {
          const float x = 0.1 * i;
          const mcl_3dl::Vec3 pos(x, 0, 0);
          const std::vector<mcl_3dl::Vec3> origins = {pos};
          const mcl_3dl::LidarMeasurementResult v = model.measure(
              kdtree, pc, origins,
              mcl_3dl::State6DOF(pos, mcl_3dl::Quat()));
          if (v.likelihood < 1.0 / 8)
            std::cerr << "_";
          else if (v.likelihood < 2.0 / 8)
            std::cerr << "▁";
          else if (v.likelihood < 3.0 / 8)
            std::cerr << "▂";
          else if (v.likelihood < 4.0 / 8)
            std::cerr << "▃";
          else if (v.likelihood < 5.0 / 8)
            std::cerr << "▄";
          else if (v.likelihood < 6.0 / 8)
            std::cerr << "▅";
          else if (v.likelihood < 7.0 / 8)
            std::cerr << "▆";
          else
            std::cerr << "▇";
        }
        std::cerr << std::endl;
        for (int i = -50; i < 50; i++)
        {
          const float x = 0.1 * i;
          const mcl_3dl::Vec3 p(x, 0, 0);
          mcl_3dl::Raycast<mcl_3dl::LidarMeasurementModelBeam::PointType>::CastResult result;
          const mcl_3dl::LidarMeasurementModelBeam::BeamStatus s = model.getBeamStatus(
              kdtree, mcl_3dl::Vec3(), p, result);
          switch (s)
          {
            case mcl_3dl::LidarMeasurementModelBeam::BeamStatus::SHORT:
              std::cerr << "s";
              break;
            case mcl_3dl::LidarMeasurementModelBeam::BeamStatus::HIT:
              std::cerr << "*";
              break;
            case mcl_3dl::LidarMeasurementModelBeam::BeamStatus::LONG:
              std::cerr << "l";
              break;
            case mcl_3dl::LidarMeasurementModelBeam::BeamStatus::TOTAL_REFLECTION:
              std::cerr << "t";
              break;
          }
        }
        std::cerr << std::endl;
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_beam_likelihood");

  return RUN_ALL_TESTS();
}

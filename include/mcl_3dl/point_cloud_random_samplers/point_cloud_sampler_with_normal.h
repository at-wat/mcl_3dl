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

#ifndef MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_SAMPLER_WITH_NORMAL_H
#define MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_SAMPLER_WITH_NORMAL_H

#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <mcl_3dl/point_cloud_random_sampler.h>
#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
template <class POINT_TYPE>
class PointCloudSamplerWithNormal : public PointCloudRandomSampler<POINT_TYPE>
{
private:
  using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  std::shared_ptr<std::default_random_engine> engine_;
  State6DOF mean_;
  Matrix eigen_vectors_;
  Vector eigen_values_;
  double perform_weighting_ratio_;
  double max_weight_ratio_;
  double max_weight_;
  double normal_search_range_;

public:
  explicit PointCloudSamplerWithNormal(const unsigned int random_seed = std::random_device()())
    : engine_(std::make_shared<std::default_random_engine>(random_seed))
    , perform_weighting_ratio_(2.0)
    , max_weight_ratio_(5.0)
    , max_weight_(10.0)
    , normal_search_range_(0.4)
  {
  }

  void loadConfig(const ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, "random_sampler_with_normal");
    pnh.param("perform_weighting_ratio", perform_weighting_ratio_, 2.0);
    pnh.param("max_weight_ratio", max_weight_ratio_, 5.0);
    pnh.param("max_weight", max_weight_, 5.0);
    pnh.param("normal_search_range", normal_search_range_, 0.4);
  }

  void setParameters(const double perform_weighting_ratio, const double max_weight_ratio,
                     const double max_weight, const double normal_search_range)
  {
    perform_weighting_ratio_ = perform_weighting_ratio;
    max_weight_ratio_ = max_weight_ratio;
    max_weight_ = max_weight;
    normal_search_range_ = normal_search_range;
  }

  void setParticleStatistics(const State6DOF& mean, const std::vector<State6DOF>& covariances)
  {
    mean_ = mean;
    Matrix pos_cov(3, 3);
    for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        pos_cov(i, j) = std::abs(covariances[i][j]);
      }
    }
    const Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(pos_cov);
    eigen_vectors_ = eigen_solver.eigenvectors();
    eigen_values_ = eigen_solver.eigenvalues();
  }

  typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
      const typename pcl::PointCloud<POINT_TYPE>::ConstPtr& pc,
      const size_t num) const final
  {
    const ros::WallTime start_timestamp = ros::WallTime::now();

    typename pcl::PointCloud<POINT_TYPE>::Ptr output(new pcl::PointCloud<POINT_TYPE>);
    output->header = pc->header;

    if ((pc->points.size() == 0) || (num == 0))
    {
      return output;
    }
    if (pc->size() <= num)
    {
      *output = *pc;
      return output;
    }

    const double eigen_value_ratio = std::sqrt(eigen_values_[2] / eigen_values_[1]);

    double max_weight = 1.0;
    if (eigen_value_ratio < perform_weighting_ratio_)
    {
      max_weight = 1.0;
    }
    else if (eigen_value_ratio > max_weight_ratio_)
    {
      max_weight = max_weight_;
    }
    else
    {
      const double weight_ratio =
          (eigen_value_ratio - perform_weighting_ratio_) / (max_weight_ratio_ - perform_weighting_ratio_);
      max_weight = 1.0 + (max_weight_ - 1.0) * weight_ratio;
    }
    const mcl_3dl::Vec3 fpc_global(eigen_vectors_(0, 2), eigen_vectors_(1, 2), eigen_vectors_(2, 2));
    const mcl_3dl::Vec3 fpc_local = mean_.rot_.inv() * fpc_global;
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    typename pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>());
    ne.setInputCloud(pc);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_search_range_);
    ne.compute(*cloud_normals);

    const ros::WallTime compute_normal_timestamp = ros::WallTime::now();
    std::vector<double> cumulative_weight(cloud_normals->points.size(), 0.0);
    for (size_t i = 0; i < cloud_normals->points.size(); i++)
    {
      double weight = 1.0;
      const auto& normal = cloud_normals->points[i];
      if (!std::isnan(normal.normal_x) && !std::isnan(normal.normal_y) && !std::isnan(normal.normal_z))
      {
        double acos_angle = std::abs(normal.normal_x * fpc_local.x_ +
                                     normal.normal_y * fpc_local.y_ +
                                     normal.normal_z * fpc_local.z_);
        // Avoid that std::acos() returns nan because of calculation errors
        if (acos_angle > 1.0)
        {
          acos_angle = 1.0;
        }
        const double angle = std::acos(acos_angle);
        weight = 1.0 + (max_weight - 1.0) * ((M_PI / 2 - angle) / (M_PI / 2));
      }
      cumulative_weight[i] = weight + ((i == 0) ? 0.0 : cumulative_weight[i - 1]);
    }
    std::uniform_real_distribution<double> ud(0, cumulative_weight.back());
    // Use unordered_set to avoid duplication
    std::unordered_set<size_t> selected_ids;
    while (true)
    {
      const double random_value = ud(*engine_);
      auto it = std::lower_bound(cumulative_weight.begin(), cumulative_weight.end(), random_value);
      const size_t n = it - cumulative_weight.begin();
      selected_ids.insert(n);
      if (selected_ids.size() >= num)
      {
        break;
      }
    }
    output->points.reserve(num);
    for (const auto& index : selected_ids)
    {
      output->push_back(pc->points[index]);
    }

    const ros::WallTime final_timestamp = ros::WallTime::now();
    ROS_DEBUG("PointCloudSamplerWithNormal::sample() computation time: %f[s] (Normal calculation: %f[s])",
              (final_timestamp - start_timestamp).toSec(), (compute_normal_timestamp - start_timestamp).toSec());
    ROS_DEBUG("Chosen eigen vector: (%f, %f, %f), max weight: %f",
              eigen_vectors_(0, 2), eigen_vectors_(1, 2), eigen_vectors_(2, 2), max_weight);
    return output;
  }
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_POINT_CLOUD_RANDOM_SAMPLERS_POINT_CLOUD_SAMPLER_WITH_NORMAL_H

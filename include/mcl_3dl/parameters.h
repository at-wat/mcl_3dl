/*
 * Copyright (c) 2016-2018, the mcl_3dl authors
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
#ifndef MCL_3DL_PARAMETERS_H
#define MCL_3DL_PARAMETERS_H

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <mcl_3dl/MCL3DLParamsConfig.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class PointCloudSamplerWithNormalParameters
{
public:
  PointCloudSamplerWithNormalParameters()
    : perform_weighting_ratio_(2.0)
    , max_weight_ratio_(5.0)
    , max_weight_(5.0)
    , normal_search_range_(0.4)
  {
  }

  double perform_weighting_ratio_;
  double max_weight_ratio_;
  double max_weight_;
  double normal_search_range_;
};

class LidarMeasurementModelLikelihoodParameters
{
public:
  LidarMeasurementModelLikelihoodParameters()
    : num_points_default_(96)
    , num_points_global_(8)
    , clip_far_(10.0)
    , clip_near_(0.5)
    , clip_z_min_(-2.0)
    , clip_z_max_(2.0)
    , match_weight_(5.0)
    , match_dist_min_(0.2)
    , match_dist_flat_(0.05)
  {
  }

  size_t num_points_default_;
  size_t num_points_global_;
  float clip_far_;
  float clip_near_;
  float clip_z_min_;
  float clip_z_max_;
  float match_weight_;
  float match_dist_min_;
  float match_dist_flat_;
};

class LidarMeasurementModelBeamParameters
{
public:
  LidarMeasurementModelBeamParameters()
    : map_grid_x_(0.1)
    , map_grid_y_(0.1)
    , map_grid_z_(0.1)
    , num_points_default_(3)
    , num_points_global_(0)
    , clip_far_(4.0)
    , clip_near_(0.5)
    , clip_z_min_(-2.0)
    , clip_z_max_(2.0)
    , beam_likelihood_min_(0.2)
    , ang_total_ref_(M_PI / 6.0)
    , filter_label_max_(static_cast<int>(0xFFFFFFFF))
    , hit_range_(0.3)
    , add_penalty_short_only_mode_(true)
    , use_raycast_using_dda_(false)
    , ray_angle_half_(0.25 * M_PI / 180.0)
    , dda_grid_size_(0.2)
  {
  }

  float map_grid_x_;
  float map_grid_y_;
  float map_grid_z_;
  size_t num_points_default_;
  size_t num_points_global_;
  float clip_far_;
  float clip_near_;
  float clip_z_min_;
  float clip_z_max_;
  float beam_likelihood_min_;
  float ang_total_ref_;
  uint32_t filter_label_max_;
  float hit_range_;
  bool add_penalty_short_only_mode_;
  bool use_raycast_using_dda_;
  float ray_angle_half_;
  float dda_grid_size_;
};

class Parameters
{
public:
  Parameters();
  bool load(ros::NodeHandle& nh);

  bool fake_imu_, fake_odom_;
  double map_downsample_x_;
  double map_downsample_y_;
  double map_downsample_z_;
  double update_downsample_x_;
  double update_downsample_y_;
  double update_downsample_z_;
  double map_grid_min_;
  double map_grid_max_;
  double global_localization_grid_;
  int global_localization_div_yaw_;
  double downsample_x_;
  double downsample_y_;
  double downsample_z_;
  double resample_var_x_;
  double resample_var_y_;
  double resample_var_z_;
  double resample_var_roll_;
  double resample_var_pitch_;
  double resample_var_yaw_;
  double expansion_var_x_;
  double expansion_var_y_;
  double expansion_var_z_;
  double expansion_var_roll_;
  double expansion_var_pitch_;
  double expansion_var_yaw_;
  double match_ratio_thresh_;
  double jump_dist_;
  double jump_ang_;
  double fix_dist_;
  double fix_ang_;
  double odom_err_lin_lin_;
  double odom_err_lin_ang_;
  double odom_err_ang_lin_;
  double odom_err_ang_ang_;
  std::shared_ptr<ros::Duration> map_update_interval_;
  int num_particles_;
  int skip_measure_;
  int accum_cloud_;
  int total_accum_cloud_max_;
  double match_output_dist_;
  double unmatch_output_dist_;
  double bias_var_dist_;
  double bias_var_ang_;
  double acc_var_;
  double odom_err_integ_lin_tc_;
  double odom_err_integ_lin_sigma_;
  double odom_err_integ_ang_tc_;
  double odom_err_integ_ang_sigma_;
  std::shared_ptr<ros::Duration> match_output_interval_;
  std::shared_ptr<ros::Duration> tf_tolerance_;
  double lpf_step_;
  double acc_lpf_step_;
  std::array<float, 4> dist_weight_;
  bool output_pcd_;
  bool publish_tf_;
  double map_chunk_;
  std::map<std::string, std::string> frame_ids_;
  std::array<float, 3> std_warn_thresh_;
  State6DOF initial_pose_;
  State6DOF initial_pose_std_;
  bool use_random_sampler_with_normal_;
  std::shared_ptr<PointCloudSamplerWithNormalParameters> random_sampler_with_normal_params_;
  std::shared_ptr<LidarMeasurementModelLikelihoodParameters> lidar_measurement_likelihood_params_;
  std::shared_ptr<LidarMeasurementModelBeamParameters> lidar_measurement_beam_params_;

private:
  std::unique_ptr<dynamic_reconfigure::Server<MCL3DLParamsConfig>> parameter_server_;
  void cbParameter(const MCL3DLParamsConfig& config, const uint32_t /* level */);
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_PARAMETERS_H

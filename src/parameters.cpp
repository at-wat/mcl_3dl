/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <mcl_3dl/MCL3DLParamsConfig.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl_compat/compatibility.h>

namespace mcl_3dl
{
Parameters::Parameters()
  : random_sampler_with_normal_params_(std::make_shared<PointCloudSamplerWithNormalParameters>())
  , lidar_measurement_likelihood_params_(std::make_shared<LidarMeasurementModelLikelihoodParameters>())
  , lidar_measurement_beam_params_(std::make_shared<LidarMeasurementModelBeamParameters>())
{
}

bool Parameters::load(ros::NodeHandle& pnh)
{
  pnh.param("fake_imu", fake_imu_, false);
  pnh.param("fake_odom", fake_odom_, false);
  if (fake_imu_ && fake_odom_)
  {
    ROS_ERROR("One of IMU and Odometry must be enabled");
    return false;
  }

  pnh.param("map_frame", frame_ids_["map"], std::string("map"));
  pnh.param("robot_frame", frame_ids_["base_link"], std::string("base_link"));
  pnh.param("odom_frame", frame_ids_["odom"], std::string("odom"));
  pnh.param("floor_frame", frame_ids_["floor"], std::string("floor"));

  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/clip_near", "clip_near");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/clip_far", "clip_far");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/clip_z_min", "clip_z_min");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/clip_z_max", "clip_z_max");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/match_dist_min", "match_dist_min");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/match_dist_flat", "match_dist_flat");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/match_weight", "match_weight");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/num_points", "num_points");
  mcl_3dl_compat::paramRename<double>(pnh, "likelihood/num_points_global", "num_points_global");

  mcl_3dl_compat::paramRename<double>(pnh, "beam/clip_near", "clip_beam_near");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/clip_far", "clip_beam_far");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/clip_z_min", "clip_beam_z_min");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/clip_z_max", "clip_beam_z_max");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/num_points", "num_points_beam");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/beam_likelihood", "beam_likelihood");
  mcl_3dl_compat::paramRename<double>(pnh, "beam/ang_total_ref", "ang_total_ref");

  pnh.param("map_downsample_x", map_downsample_x_, 0.1);
  pnh.param("map_downsample_y", map_downsample_y_, 0.1);
  pnh.param("map_downsample_z", map_downsample_z_, 0.1);
  pnh.param("downsample_x", downsample_x_, 0.1);
  pnh.param("downsample_y", downsample_y_, 0.1);
  pnh.param("downsample_z", downsample_z_, 0.05);
  map_grid_min_ =
      std::min(
          std::min(map_downsample_x_, map_downsample_y_),
          map_downsample_z_);
  map_grid_max_ =
      std::max(
          std::max(map_downsample_x_, map_downsample_y_),
          map_downsample_z_);

  pnh.param("update_downsample_x", update_downsample_x_, 0.3);
  pnh.param("update_downsample_y", update_downsample_y_, 0.3);
  pnh.param("update_downsample_z", update_downsample_z_, 0.3);

  double map_update_interval_t;
  pnh.param("map_update_interval_interval", map_update_interval_t, 2.0);
  map_update_interval_.reset(new ros::Duration(map_update_interval_t));

  pnh.param("dist_weight_x", dist_weight_[0], 1.0f);
  pnh.param("dist_weight_y", dist_weight_[1], 1.0f);
  pnh.param("dist_weight_z", dist_weight_[2], 5.0f);
  dist_weight_[3] = 0.0;

  pnh.param("global_localization_grid_lin", global_localization_grid_, 0.3);
  double grid_ang;
  pnh.param("global_localization_grid_ang", grid_ang, 0.524);
  global_localization_div_yaw_ = std::lround(2 * M_PI / grid_ang);

  pnh.param("num_particles", num_particles_, 64);

  pnh.param("resample_var_x", resample_var_x_, 0.05);
  pnh.param("resample_var_y", resample_var_y_, 0.05);
  pnh.param("resample_var_z", resample_var_z_, 0.05);
  pnh.param("resample_var_roll", resample_var_roll_, 0.05);
  pnh.param("resample_var_pitch", resample_var_pitch_, 0.05);
  pnh.param("resample_var_yaw", resample_var_yaw_, 0.05);
  pnh.param("expansion_var_x", expansion_var_x_, 0.2);
  pnh.param("expansion_var_y", expansion_var_y_, 0.2);
  pnh.param("expansion_var_z", expansion_var_z_, 0.2);
  pnh.param("expansion_var_roll", expansion_var_roll_, 0.05);
  pnh.param("expansion_var_pitch", expansion_var_pitch_, 0.05);
  pnh.param("expansion_var_yaw", expansion_var_yaw_, 0.05);
  pnh.param("match_ratio_thresh", match_ratio_thresh_, 0.0);

  pnh.param("odom_err_lin_lin", odom_err_lin_lin_, 0.10);
  pnh.param("odom_err_lin_ang", odom_err_lin_ang_, 0.05);
  pnh.param("odom_err_ang_lin", odom_err_ang_lin_, 0.05);
  pnh.param("odom_err_ang_ang", odom_err_ang_ang_, 0.05);

  pnh.param("odom_err_integ_lin_tc", odom_err_integ_lin_tc_, 10.0);
  pnh.param("odom_err_integ_lin_sigma", odom_err_integ_lin_sigma_, 100.0);
  pnh.param("odom_err_integ_ang_tc", odom_err_integ_ang_tc_, 10.0);
  pnh.param("odom_err_integ_ang_sigma", odom_err_integ_ang_sigma_, 100.0);

  pnh.param("lpf_step", lpf_step_, 16.0);
  pnh.param("acc_lpf_step", acc_lpf_step_, 128.0);

  pnh.param("acc_var", acc_var_, M_PI / 4.0);  // 45 deg

  pnh.param("jump_dist", jump_dist_, 1.0);
  pnh.param("jump_ang", jump_ang_, 1.57);
  pnh.param("fix_dist", fix_dist_, 0.2);
  pnh.param("fix_ang", fix_ang_, 0.1);
  pnh.param("bias_var_dist", bias_var_dist_, 2.0);
  pnh.param("bias_var_ang", bias_var_ang_, 1.57);

  pnh.param("skip_measure", skip_measure_, 1);
  pnh.param("accum_cloud", accum_cloud_, 1);
  pnh.param("total_accum_cloud_max", total_accum_cloud_max_, accum_cloud_ * 10);

  double match_output_interval_t;
  pnh.param("match_output_interval_interval", match_output_interval_t, 0.2);
  match_output_interval_.reset(new ros::Duration(match_output_interval_t));

  double tf_tolerance_t;
  pnh.param("tf_tolerance", tf_tolerance_t, 0.05);
  tf_tolerance_.reset(new ros::Duration(tf_tolerance_t));

  pnh.param("match_output_dist", match_output_dist_, 0.1);
  pnh.param("unmatch_output_dist", unmatch_output_dist_, 0.5);

  pnh.param("publish_tf", publish_tf_, true);
  pnh.param("output_pcd", output_pcd_, false);

  const float float_max = std::numeric_limits<float>::max();
  pnh.param("std_warn_thresh_xy", std_warn_thresh_[0], float_max);
  pnh.param("std_warn_thresh_z", std_warn_thresh_[1], float_max);
  pnh.param("std_warn_thresh_yaw", std_warn_thresh_[2], float_max);

  pnh.param("map_chunk", map_chunk_, 20.0);

  double x, y, z;
  double roll, pitch, yaw;
  double v_x, v_y, v_z;
  double v_roll, v_pitch, v_yaw;
  pnh.param("init_x", x, 0.0);
  pnh.param("init_y", y, 0.0);
  pnh.param("init_z", z, 0.0);
  pnh.param("init_roll", roll, 0.0);
  pnh.param("init_pitch", pitch, 0.0);
  pnh.param("init_yaw", yaw, 0.0);
  pnh.param("init_var_x", v_x, 2.0);
  pnh.param("init_var_y", v_y, 2.0);
  pnh.param("init_var_z", v_z, 0.5);
  pnh.param("init_var_roll", v_roll, 0.1);
  pnh.param("init_var_pitch", v_pitch, 0.1);
  pnh.param("init_var_yaw", v_yaw, 0.5);
  initial_pose_ = State6DOF(
      Vec3(x, y, z),
      Quat(Vec3(roll, pitch, yaw)));
  initial_pose_std_ = State6DOF(
      Vec3(v_x, v_y, v_z),
      Vec3(v_roll, v_pitch, v_yaw));

  pnh.param("use_random_sampler_with_normal", use_random_sampler_with_normal_, false);

  if (use_random_sampler_with_normal_)
  {
    ros::NodeHandle rs_pnh(pnh, "random_sampler_with_normal");
    rs_pnh.param("perform_weighting_ratio", random_sampler_with_normal_params_->perform_weighting_ratio_, 2.0);
    rs_pnh.param("max_weight_ratio", random_sampler_with_normal_params_->max_weight_ratio_, 5.0);
    rs_pnh.param("max_weight", random_sampler_with_normal_params_->max_weight_, 5.0);
    rs_pnh.param("normal_search_range", random_sampler_with_normal_params_->normal_search_range_, 0.4);
  }

  {
    ros::NodeHandle lml_pnh(pnh, "likelihood");

    int num_points, num_points_global;
    lml_pnh.param("num_points", num_points, 96);
    lml_pnh.param("num_points_global", num_points_global, 8);
    lidar_measurement_likelihood_params_->num_points_default_ = num_points;
    lidar_measurement_likelihood_params_->num_points_global_ = num_points_global;

    double clip_near, clip_far;
    lml_pnh.param("clip_near", clip_near, 0.5);
    lml_pnh.param("clip_far", clip_far, 10.0);
    lidar_measurement_likelihood_params_->clip_near_ = clip_near;
    lidar_measurement_likelihood_params_->clip_far_ = clip_far;

    double clip_z_min, clip_z_max;
    lml_pnh.param("clip_z_min", clip_z_min, -2.0);
    lml_pnh.param("clip_z_max", clip_z_max, 2.0);
    lidar_measurement_likelihood_params_->clip_z_min_ = clip_z_min;
    lidar_measurement_likelihood_params_->clip_z_max_ = clip_z_max;

    double match_weight;
    lml_pnh.param("match_weight", match_weight, 5.0);
    lidar_measurement_likelihood_params_->match_weight_ = match_weight;

    double match_dist_min, match_dist_flat;
    lml_pnh.param("match_dist_min", match_dist_min, 0.2);
    lml_pnh.param("match_dist_flat", match_dist_flat, 0.05);
    lidar_measurement_likelihood_params_->match_dist_min_ = match_dist_min;
    lidar_measurement_likelihood_params_->match_dist_flat_ = match_dist_flat;
  }

  {
    ros::NodeHandle lmb_pnh(pnh, "beam");

    lidar_measurement_beam_params_->map_grid_x_ = map_downsample_x_;
    lidar_measurement_beam_params_->map_grid_y_ = map_downsample_y_;
    lidar_measurement_beam_params_->map_grid_z_ = map_downsample_z_;

    int num_points, num_points_global;
    lmb_pnh.param("num_points", num_points, 3);
    lmb_pnh.param("num_points_global", num_points_global, 0);
    lidar_measurement_beam_params_->num_points_default_ = num_points;
    lidar_measurement_beam_params_->num_points_global_ = num_points_global;

    double clip_near, clip_far;
    lmb_pnh.param("clip_near", clip_near, 0.5);
    lmb_pnh.param("clip_far", clip_far, 4.0);
    lidar_measurement_beam_params_->clip_near_ = clip_near;
    lidar_measurement_beam_params_->clip_far_ = clip_far;

    double clip_z_min, clip_z_max;
    lmb_pnh.param("clip_z_min", clip_z_min, -2.0);
    lmb_pnh.param("clip_z_max", clip_z_max, 2.0);
    lidar_measurement_beam_params_->clip_z_min_ = clip_z_min;
    lidar_measurement_beam_params_->clip_z_max_ = clip_z_max;

    double beam_likelihood;
    lmb_pnh.param("beam_likelihood", beam_likelihood, 0.2);
    lidar_measurement_beam_params_->beam_likelihood_min_ = beam_likelihood;

    double ang_total_ref;
    lmb_pnh.param("ang_total_ref", ang_total_ref, M_PI / 6.0);
    lidar_measurement_beam_params_->ang_total_ref_ = ang_total_ref;

    int filter_label_max;
    lmb_pnh.param("filter_label_max", filter_label_max, static_cast<int>(0xFFFFFFFF));
    lidar_measurement_beam_params_->filter_label_max_ = filter_label_max;

    lmb_pnh.param("add_penalty_short_only_mode", lidar_measurement_beam_params_->add_penalty_short_only_mode_, true);
    double hit_range;
    lmb_pnh.param("hit_range", hit_range, 0.3);
    lidar_measurement_beam_params_->hit_range_ = hit_range;

    lmb_pnh.param("use_raycast_using_dda", lidar_measurement_beam_params_->use_raycast_using_dda_, false);
    if (lidar_measurement_beam_params_->use_raycast_using_dda_)
    {
      double ray_angle_half;
      lmb_pnh.param("ray_angle_half", ray_angle_half, 0.25 * M_PI / 180.0);
      lidar_measurement_beam_params_->ray_angle_half_ = ray_angle_half;

      double dda_grid_size;
      lmb_pnh.param("dda_grid_size", dda_grid_size, 0.2);
      const double grid_size_max = std::max(
          {
              lidar_measurement_beam_params_->map_grid_x_,
              lidar_measurement_beam_params_->map_grid_y_,
              lidar_measurement_beam_params_->map_grid_z_,
          });  // NOLINT(whitespace/braces)
      if (dda_grid_size < grid_size_max)
      {
        ROS_WARN("dda_grid_size must be larger than grid size. New value: %f", grid_size_max);
        dda_grid_size = grid_size_max;
      }
      lidar_measurement_beam_params_->dda_grid_size_ = dda_grid_size;
    }
  }

  parameter_server_.reset(new dynamic_reconfigure::Server<MCL3DLParamsConfig>(pnh));
  parameter_server_->setCallback(
      boost::bind(&Parameters::cbParameter, this, boost::placeholders::_1, boost::placeholders::_2));

  return true;
}

void Parameters::cbParameter(const MCL3DLParamsConfig& config, const uint32_t /* level */)
{
}

}  // namespace mcl_3dl

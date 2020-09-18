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
#include <limits>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <mcl_3dl/parameters.h>

#include <mcl_3dl_compat/compatibility.h>

namespace mcl_3dl
{
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

  return true;
}
}  // namespace mcl_3dl

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

#include <map>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <mcl_3dl/quat.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
class Parameters
{
public:
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
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_PARAMETERS_H

/*
 * Copyright (c) 2016-2021, the mcl_3dl authors
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

#ifndef MCL_3DL_MCL_3DL_H
#define MCL_3DL_MCL_3DL_H

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mcl_3dl_msgs/ResizeParticle.h>
#include <mcl_3dl_msgs/Status.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/cloud_accum.h>
#include <mcl_3dl/filter.h>
#include <mcl_3dl/filter_vec3.h>
#include <mcl_3dl/imu_measurement_model_base.h>
#include <mcl_3dl/imu_measurement_models/imu_measurement_model_gravity.h>
#include <mcl_3dl/lidar_measurement_model_base.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/motion_prediction_model_base.h>
#include <mcl_3dl/motion_prediction_models/motion_prediction_model_differential_drive.h>
#include <mcl_3dl/nd.h>
#include <mcl_3dl/noise_generators/multivariate_noise_generator.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_sampler_with_normal.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_uniform_sampler.h>
#include <mcl_3dl/point_conversion.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

#include <mcl_3dl_compat/compatibility.h>

namespace mcl_3dl
{
class MCL3dlNode
{
protected:
  using PointType = mcl_3dl::PointXYZIL;
  std::shared_ptr<pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat, std::default_random_engine>> pf_;

  class MyPointRepresentation : public pcl::PointRepresentation<PointType>
  {
    using pcl::PointRepresentation<PointType>::nr_dimensions_;

  public:
    inline MyPointRepresentation()
    {
      nr_dimensions_ = 3;
    }

    inline virtual void copyToFloatArray(const PointType& p, float* out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
    }
  };

  void cbMapcloudUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cbPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void accumClear();
  bool accumCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void measure();
  void cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void cbImu(const sensor_msgs::Imu::ConstPtr& msg);
  bool cbResizeParticle(mcl_3dl_msgs::ResizeParticleRequest& request,
                        mcl_3dl_msgs::ResizeParticleResponse& response);
  bool cbExpansionReset(std_srvs::TriggerRequest& request,
                        std_srvs::TriggerResponse& response);
  bool cbGlobalLocalization(std_srvs::TriggerRequest& request,
                            std_srvs::TriggerResponse& response);
  void publishParticles();
  float getEntropy();
  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

public:
  inline MCL3dlNode()
    : pnh_("~")
    , tfl_(tfbuf_)
    , cnt_measure_(0)
    , global_localization_fix_cnt_(0)
    , engine_(seed_gen_())
  {
  }
  bool configure();
  ~MCL3dlNode();
  void cbMapUpdateTimer(const ros::TimerEvent& event);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_mapcloud_;
  ros::Subscriber sub_mapcloud_update_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_landmark_;
  ros::Publisher pub_particle_;
  ros::Publisher pub_mapcloud_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_matched_;
  ros::Publisher pub_unmatched_;
  ros::Publisher pub_debug_marker_;
  ros::Publisher pub_status_;
  ros::Timer map_update_timer_;
  ros::ServiceServer srv_particle_size_;
  ros::ServiceServer srv_global_localization_;
  ros::ServiceServer srv_expansion_reset_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::TransformBroadcaster tfb_;

  std::shared_ptr<FilterVec3> f_pos_;
  std::shared_ptr<FilterVec3> f_ang_;
  std::shared_ptr<FilterVec3> f_acc_;
  std::shared_ptr<Filter> localize_rate_;
  ros::Time localized_last_;
  ros::Duration tf_tolerance_base_;

  Parameters params_;

  ros::Time match_output_last_;
  ros::Time odom_last_;
  bool has_map_;
  bool has_odom_;
  bool has_imu_;
  State6DOF odom_;
  State6DOF odom_prev_;
  State6DOF state_prev_;
  ros::Time imu_last_;
  size_t cnt_measure_;
  Quat imu_quat_;
  size_t global_localization_fix_cnt_;
  diagnostic_updater::Updater diag_updater_;
  mcl_3dl_msgs::Status status_;

  MyPointRepresentation point_rep_;

  pcl::PointCloud<PointType>::Ptr pc_map_;
  pcl::PointCloud<PointType>::Ptr pc_map2_;
  pcl::PointCloud<PointType>::Ptr pc_update_;
  pcl::PointCloud<PointType>::Ptr pc_all_accum_;
  ChunkedKdtree<PointType>::Ptr kdtree_;

  CloudAccumulationLogicBase::Ptr accum_;
  pcl::PointCloud<PointType>::Ptr pc_local_accum_;
  std::vector<std_msgs::Header> pc_accum_header_;

  std::map<
      std::string,
      LidarMeasurementModelBase::Ptr> lidar_measurements_;
  ImuMeasurementModelBase::Ptr imu_measurement_model_;
  MotionPredictionModelBase::Ptr motion_prediction_model_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_MCL_3DL_H

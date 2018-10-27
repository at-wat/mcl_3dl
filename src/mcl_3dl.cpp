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

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <mcl_3dl_msgs/ResizeParticle.h>
#include <mcl_3dl_msgs/Status.h>
#include <std_srvs/Trigger.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/filter.h>
#include <mcl_3dl/lidar_measurement_model_base.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/nd.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl/pf.h>
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
  std::shared_ptr<pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat>> pf_;

  class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointXYZI>
  {
    using pcl::PointRepresentation<pcl::PointXYZI>::nr_dimensions_;

  public:
    MyPointRepresentation()
    {
      nr_dimensions_ = 3;
    }

    virtual void copyToFloatArray(const pcl::PointXYZI &p, float *out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
    }
  };
  void cbMapcloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    ROS_INFO("map received");
    pcl::PointCloud<pcl::PointXYZI> pc_tmp;
    pcl::fromROSMsg(*msg, pc_tmp);

    if (pc_tmp.points.size() == 0)
    {
      ROS_ERROR("Empty map received.");
      has_map_ = false;
      return;
    }

    pc_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_map2_.reset();
    pc_update_.reset();
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_tmp.makeShared());
    ds.setLeafSize(params_.map_downsample_x_, params_.map_downsample_y_, params_.map_downsample_z_);
    ds.filter(*pc_map_);
    pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_all_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    frame_num_ = 0;
    has_map_ = true;

    ROS_INFO("map original: %d points", (int)pc_tmp.points.size());
    ROS_INFO("map reduced: %d points", (int)pc_map_->points.size());

    cbMapUpdateTimer(ros::TimerEvent());
  }
  void cbMapcloudUpdate(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    ROS_INFO("map_update received");
    pcl::PointCloud<pcl::PointXYZI> pc_tmp;
    pcl::fromROSMsg(*msg, pc_tmp);

    pc_update_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_tmp.makeShared());
    ds.setLeafSize(params_.update_downsample_x_, params_.update_downsample_y_, params_.update_downsample_z_);
    ds.filter(*pc_update_);
  }

  void cbPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    geometry_msgs::PoseStamped pose_in, pose;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;
    try
    {
      tfl_.waitForTransform(frame_ids_["map"], pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(1.0));
      tfl_.transformPose(frame_ids_["map"], pose_in, pose);
    }
    catch (tf::TransformException &e)
    {
      return;
    }
    pf_->init(
        State6DOF(
            Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            Quat(pose.pose.orientation.x,
                 pose.pose.orientation.y,
                 pose.pose.orientation.z,
                 pose.pose.orientation.w)),
        State6DOF(
            Vec3(msg->pose.covariance[0],
                 msg->pose.covariance[6 * 1 + 1],
                 msg->pose.covariance[6 * 2 + 2]),
            Vec3(msg->pose.covariance[6 * 3 + 3],
                 msg->pose.covariance[6 * 4 + 4],
                 msg->pose.covariance[6 * 5 + 5])));
    pc_update_.reset();
    auto integ_reset_func = [](State6DOF &s)
    {
      s.odom_err_integ_lin = Vec3();
      s.odom_err_integ_ang = Vec3();
    };
    pf_->predict(integ_reset_func);
  }

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_ =
        State6DOF(
            Vec3(msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z),
            Quat(msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z,
                 msg->pose.pose.orientation.w));
    if (!has_odom_)
    {
      odom_prev_ = odom_;
      odom_last_ = msg->header.stamp;
      has_odom_ = true;
      return;
    }
    const float dt = (msg->header.stamp - odom_last_).toSec();
    if (dt < 0.0 || dt > 5.0)
    {
      ROS_WARN("Detected time jump in odometry. Resetting.");
      has_odom_ = false;
      return;
    }
    else if (dt > 0.05)
    {
      const Vec3 v = odom_prev_.rot.inv() * (odom_.pos - odom_prev_.pos);
      const Quat r = odom_prev_.rot.inv() * odom_.rot;
      Vec3 axis;
      float ang;
      r.getAxisAng(axis, ang);

      const float trans = v.norm();
      auto prediction_func = [this, &v, &r, axis, ang, trans, &dt](State6DOF &s)
      {
        const Vec3 diff = v * (1.0 + s.noise_ll_) + Vec3(s.noise_al_ * ang, 0.0, 0.0);
        s.odom_err_integ_lin += (diff - v);
        s.pos += s.rot * diff;
        const float yaw_diff = s.noise_la_ * trans + s.noise_aa_ * ang;
        s.rot = Quat(Vec3(0.0, 0.0, 1.0), yaw_diff) * s.rot * r;
        s.rot.normalize();
        s.odom_err_integ_ang += Vec3(0.0, 0.0, yaw_diff);
        s.odom_err_integ_lin *= (1.0 - dt / params_.odom_err_integ_lin_tc_);
        s.odom_err_integ_ang *= (1.0 - dt / params_.odom_err_integ_ang_tc_);
      };
      pf_->predict(prediction_func);
      odom_last_ = msg->header.stamp;
      odom_prev_ = odom_;
    }
  }
  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    mcl_3dl_msgs::Status status;
    status.header.stamp = ros::Time::now();
    status.status = mcl_3dl_msgs::Status::NORMAL;

    if (!has_map_)
      return;
    if (frames_.find(msg->header.frame_id) == frames_.end())
    {
      frames_[msg->header.frame_id] = true;
      frames_v_.push_back(msg->header.frame_id);
    }

    sensor_msgs::PointCloud2 pc_bl;
    try
    {
      if (!pcl_ros::transformPointCloud(frame_ids_["odom"], *msg, pc_bl, tfl_))
      {
        ROS_INFO("Failed to transform pointcloud.");
        pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pc_accum_header_.clear();
        return;
      }
    }
    catch (tf::TransformException &e)
    {
      ROS_INFO("Failed to transform pointcloud: %s", e.what());
      pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pc_accum_header_.clear();
      return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc_bl, *pc_tmp);
    for (auto &p : pc_tmp->points)
    {
      p.intensity = pc_accum_header_.size();
    }
    *pc_local_accum_ += *pc_tmp;
    pc_local_accum_->header.frame_id = frame_ids_["odom"];
    pc_accum_header_.push_back(msg->header);

    if (frames_v_[frame_num_].compare(msg->header.frame_id) != 0)
      return;
    frame_num_++;
    if (frame_num_ >= frames_v_.size())
      frame_num_ = 0;

    if (frame_num_ != 0)
      return;

    cnt_accum_++;
    if (cnt_accum_ % params_.accum_cloud_ != 0)
      return;

    cnt_measure_++;
    if (cnt_measure_ % params_.skip_measure_ != 0)
    {
      pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pc_accum_header_.clear();
      return;
    }

    try
    {
      if (!pcl_ros::transformPointCloud(
              frame_ids_["base_link"], *pc_local_accum_, *pc_local_accum_, tfl_))
      {
        ROS_INFO("Failed to transform pointcloud.");
        pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pc_accum_header_.clear();
        return;
      }
    }
    catch (tf::TransformException &e)
    {
      ROS_INFO("Failed to transform pointcloud: %s", e.what());
      pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pc_accum_header_.clear();
      return;
    }
    std::vector<Vec3> origins;
    for (auto &h : pc_accum_header_)
    {
      try
      {
        tf::StampedTransform trans;
        tfl_.lookupTransform(frame_ids_["base_link"], msg->header.stamp,
                             h.frame_id, h.stamp,
                             frame_ids_["odom"], trans);
        auto origin = trans.getOrigin();
        origins.push_back(Vec3(origin.x(), origin.y(), origin.z()));
      }
      catch (tf::TransformException &e)
      {
        ROS_INFO("Failed to transform pointcloud: %s", e.what());
        pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pc_accum_header_.clear();
        return;
      }
    }

    const auto ts = boost::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_full(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_local_accum_);
    ds.setLeafSize(params_.downsample_x_, params_.downsample_y_, params_.downsample_z_);
    ds.filter(*pc_local_full);

    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr> pc_locals;
    for (auto &lm : lidar_measurements_)
    {
      lm.second->setGlobalLocalizationStatus(
          params_.num_particles_, pf_->getParticleSize());
      pc_locals[lm.first] = lm.second->filter(pc_local_full);
    }

    if (pc_locals["likelihood"]->size() == 0)
    {
      ROS_ERROR("All points are filtered out. Failed to localize.");
      return;
    }
    if (pc_locals["beam"] && pc_locals["beam"]->size() == 0)
    {
      ROS_DEBUG("All beam points are filtered out. Skipping beam model.");
    }

    float match_ratio_min = 1.0;
    float match_ratio_max = 0.0;
    NormalLikelihood<float> odom_error_lin_nd(params_.odom_err_integ_lin_sigma_);
    NormalLikelihood<float> odom_error_ang_nd(params_.odom_err_integ_ang_sigma_);
    auto measure_func = [this, &pc_locals,
                         &origins,
                         &odom_error_lin_nd,
                         &match_ratio_min, &match_ratio_max](const State6DOF &s) -> float
    {
      float likelihood = 1;
      std::map<std::string, float> qualities;
      for (auto &lm : lidar_measurements_)
      {
        const LidarMeasurementResult result = lm.second->measure(
            kdtree_, pc_locals[lm.first], origins, s);
        likelihood *= result.likelihood;
        qualities[lm.first] = result.quality;
      }
      if (match_ratio_min > qualities["likelihood"])
        match_ratio_min = qualities["likelihood"];
      if (match_ratio_max < qualities["likelihood"])
        match_ratio_max = qualities["likelihood"];

      // odometry error integration
      const float odom_error =
          odom_error_lin_nd(s.odom_err_integ_lin.norm());
      return likelihood * odom_error;
    };
    pf_->measure(measure_func);

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      auto bias_func = [](const State6DOF &s, float &p_bias) -> void
      {
        p_bias = 1.0;
      };
      pf_->bias(bias_func);
    }
    else
    {
      NormalLikelihood<float> nl_lin(params_.bias_var_dist_);
      NormalLikelihood<float> nl_ang(params_.bias_var_ang_);
      auto bias_func = [this, &nl_lin, &nl_ang](const State6DOF &s, float &p_bias) -> void
      {
        const float lin_diff = (s.pos - state_prev_.pos).norm();
        Vec3 axis;
        float ang_diff;
        (s.rot * state_prev_.rot.inv()).getAxisAng(axis, ang_diff);
        p_bias = nl_lin(lin_diff) * nl_ang(ang_diff) + 1e-6;
        assert(std::isfinite(p_bias));
      };
      pf_->bias(bias_func);
    }
    auto e = pf_->expectationBiased();
    const auto e_max = pf_->max();

    assert(std::isfinite(e.pos.x));
    assert(std::isfinite(e.pos.y));
    assert(std::isfinite(e.pos.z));
    assert(std::isfinite(e.rot.x));
    assert(std::isfinite(e.rot.y));
    assert(std::isfinite(e.rot.z));
    assert(std::isfinite(e.rot.w));

    e.rot.normalize();

    if (lidar_measurements_["beam"])
    {
      visualization_msgs::MarkerArray markers;

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle_beam(new pcl::PointCloud<pcl::PointXYZI>);
      *pc_particle_beam = *pc_locals["beam"];
      e.transform(*pc_particle_beam);
      for (auto &p : pc_particle_beam->points)
      {
        int beam_header_id = lroundf(p.intensity);
        Vec3 pos = e.pos + e.rot * origins[beam_header_id];
        Vec3 end(p.x, p.y, p.z);

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_ids_["map"];
        marker.header.stamp = msg->header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
        marker.lifetime = ros::Duration(0.2);
        marker.frame_locked = true;
        marker.points.resize(2);
        marker.points[0].x = pos.x;
        marker.points[0].y = pos.y;
        marker.points[0].z = pos.z;
        marker.points[1].x = end.x;
        marker.points[1].y = end.y;
        marker.points[1].z = end.z;
        marker.colors.resize(2);
        marker.colors[0].a = 0.5;
        marker.colors[0].r = 1.0;
        marker.colors[0].g = 0.0;
        marker.colors[0].b = 0.0;
        marker.colors[1].a = 0.2;
        marker.colors[1].r = 1.0;
        marker.colors[1].g = 0.0;
        marker.colors[1].b = 0.0;

        markers.markers.push_back(marker);
      }
      const auto beam_model =
          std::dynamic_pointer_cast<LidarMeasurementModelBeam>(
              lidar_measurements_["beam"]);
      const float sin_total_ref = beam_model->getSinTotalRef();
      for (auto &p : pc_particle_beam->points)
      {
        const int beam_header_id = lroundf(p.intensity);
        Raycast<pcl::PointXYZI> ray(
            kdtree_,
            e.pos + e.rot * origins[beam_header_id],
            Vec3(p.x, p.y, p.z),
            params_.map_grid_min_, params_.map_grid_max_);
        for (auto point : ray)
        {
          if (point.collision_)
          {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_ids_["map"];
            marker.header.stamp = msg->header.stamp;
            marker.ns = "Ray collisions";
            marker.id = markers.markers.size();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = 0;
            marker.pose.position.x = point.pos_.x;
            marker.pose.position.y = point.pos_.y;
            marker.pose.position.z = point.pos_.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
            marker.lifetime = ros::Duration(0.2);
            marker.frame_locked = true;
            marker.color.a = 0.8;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            if (point.sin_angle_ < sin_total_ref)
            {
              marker.color.a = 0.2;
            }
            markers.markers.push_back(marker);
            break;
          }
        }
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
      *pc_particle = *pc_locals["likelihood"];
      e.transform(*pc_particle);
      for (auto &p : pc_particle->points)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_ids_["map"];
        marker.header.stamp = msg->header.stamp;
        marker.ns = "Sample points";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = 0;
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = p.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.lifetime = ros::Duration(0.2);
        marker.frame_locked = true;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        markers.markers.push_back(marker);
      }

      pub_debug_marker_.publish(markers);
    }

    Vec3 map_pos;
    Quat map_rot;
    map_pos = e.pos - e.rot * odom_.rot.inv() * odom_.pos;
    map_rot = e.rot * odom_.rot.inv();

    bool jump = false;
    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      jump = true;
      state_prev_ = e;
    }
    else
    {
      Vec3 jump_axis;
      float jump_ang;
      float jump_dist = (e.pos - state_prev_.pos).norm();
      (e.rot.inv() * state_prev_.rot).getAxisAng(jump_axis, jump_ang);
      if (jump_dist > params_.jump_dist_ ||
          fabs(jump_ang) > params_.jump_ang_)
      {
        ROS_INFO("Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
        jump = true;

        auto integ_reset_func = [](State6DOF &s)
        {
          s.odom_err_integ_lin = Vec3();
          s.odom_err_integ_ang = Vec3();
        };
        pf_->predict(integ_reset_func);
      }
      state_prev_ = e;
    }
    tf::StampedTransform trans;
    if (has_odom_)
      trans.stamp_ = odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance_;
    else
      trans.stamp_ = ros::Time::now() + tf_tolerance_base_ + *params_.tf_tolerance_;
    trans.frame_id_ = frame_ids_["map"];
    trans.child_frame_id_ = frame_ids_["odom"];
    auto rpy = map_rot.getRPY();
    if (jump)
    {
      f_ang_[0]->set(rpy.x);
      f_ang_[1]->set(rpy.y);
      f_ang_[2]->set(rpy.z);
      f_pos_[0]->set(map_pos.x);
      f_pos_[1]->set(map_pos.y);
      f_pos_[2]->set(map_pos.z);
    }
    rpy.x = f_ang_[0]->in(rpy.x);
    rpy.y = f_ang_[1]->in(rpy.y);
    rpy.z = f_ang_[2]->in(rpy.z);
    map_rot.setRPY(rpy);
    map_pos.x = f_pos_[0]->in(map_pos.x);
    map_pos.y = f_pos_[1]->in(map_pos.y);
    map_pos.z = f_pos_[2]->in(map_pos.z);
    trans.setOrigin(tf::Vector3(map_pos.x, map_pos.y, map_pos.z));
    trans.setRotation(tf::Quaternion(map_rot.x, map_rot.y, map_rot.z, map_rot.w));

    std::vector<tf::StampedTransform> transforms;
    transforms.push_back(trans);

    e.rot = map_rot * odom_.rot;
    e.pos = map_pos + e.rot * odom_.rot.inv() * odom_.pos;

    assert(std::isfinite(e.pos.x));
    assert(std::isfinite(e.pos.y));
    assert(std::isfinite(e.pos.z));
    assert(std::isfinite(e.rot.x));
    assert(std::isfinite(e.rot.y));
    assert(std::isfinite(e.rot.z));
    assert(std::isfinite(e.rot.w));

    trans.frame_id_ = frame_ids_["map"];
    trans.child_frame_id_ = frame_ids_["floor"];
    trans.setOrigin(tf::Vector3(0.0, 0.0, e.pos.z));
    trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transforms.push_back(trans);

    if (publish_tf_)
      tfb_.sendTransform(transforms);

    auto cov = pf_->covariance();

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = trans.frame_id_;
    pose.pose.pose.position.x = e.pos.x;
    pose.pose.pose.position.y = e.pos.y;
    pose.pose.pose.position.z = e.pos.z;
    pose.pose.pose.orientation.x = e.rot.x;
    pose.pose.pose.orientation.y = e.rot.y;
    pose.pose.pose.orientation.z = e.rot.z;
    pose.pose.pose.orientation.w = e.rot.w;
    for (size_t i = 0; i < 36; i++)
    {
      pose.pose.covariance[i] = cov[i / 6][i % 6];
    }
    pub_pose_.publish(pose);

    {
      bool fix = false;
      Vec3 fix_axis;
      const float fix_ang = sqrtf(cov[3][3] + cov[4][4] + cov[5][5]);
      const float fix_dist = sqrtf(cov[0][0] + cov[1][1] + cov[2][2]);
      ROS_DEBUG("cov: lin %0.3f ang %0.3f", fix_dist, fix_ang);
      if (fix_dist < params_.fix_dist_ &&
          fabs(fix_ang) < params_.fix_ang_)
      {
        fix = true;
      }

      if (fix)
        ROS_DEBUG("Localization fixed");
    }

    if (output_pcd_)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
      *pc_particle = *pc_locals["likelihood"];
      e.transform(*pc_particle);
      *pc_all_accum_ += *pc_particle;
    }

    if ((msg->header.stamp - match_output_last_ > *params_.match_output_interval_ ||
         msg->header.stamp < match_output_last_ - ros::Duration(1.0)) &&
        (pub_matched_.getNumSubscribers() > 0 || pub_unmatched_.getNumSubscribers() > 0))
    {
      match_output_last_ = msg->header.stamp;

      sensor_msgs::PointCloud pc_match;
      pc_match.header.stamp = msg->header.stamp;
      pc_match.header.frame_id = frame_ids_["map"];
      sensor_msgs::PointCloud pc_unmatch;
      pc_unmatch.header.stamp = msg->header.stamp;
      pc_unmatch.header.frame_id = frame_ids_["map"];

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local(new pcl::PointCloud<pcl::PointXYZI>);
      *pc_local = *pc_local_full;

      e.transform(*pc_local);

      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      const double match_dist_sq = params_.match_output_dist_ * params_.match_output_dist_;
      for (auto &p : pc_local->points)
      {
        geometry_msgs::Point32 pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;

        if (!kdtree_->radiusSearch(p, params_.unmatch_output_dist_, id, sqdist, 1))
        {
          pc_unmatch.points.push_back(pp);
        }
        else if (sqdist[0] < match_dist_sq)
        {
          pc_match.points.push_back(pp);
        }
      }
      if (pub_matched_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::convertPointCloudToPointCloud2(pc_match, pc2);
        pub_matched_.publish(pc2);
      }
      if (pub_unmatched_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::convertPointCloudToPointCloud2(pc_unmatch, pc2);
        pub_unmatched_.publish(pc2);
      }
    }

    geometry_msgs::PoseArray pa;
    if (has_odom_)
      pa.header.stamp = odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance_;
    else
      pa.header.stamp = ros::Time::now() + tf_tolerance_base_ + *params_.tf_tolerance_;
    pa.header.frame_id = frame_ids_["map"];
    for (size_t i = 0; i < pf_->getParticleSize(); i++)
    {
      geometry_msgs::Pose pm;
      auto p = pf_->getParticle(i);
      p.rot.normalize();
      pm.position.x = p.pos.x;
      pm.position.y = p.pos.y;
      pm.position.z = p.pos.z;
      pm.orientation.x = p.rot.x;
      pm.orientation.y = p.rot.y;
      pm.orientation.z = p.rot.z;
      pm.orientation.w = p.rot.w;
      pa.poses.push_back(pm);
    }
    pub_particle_.publish(pa);

    pf_->resample(State6DOF(
        Vec3(params_.resample_var_x_,
             params_.resample_var_y_,
             params_.resample_var_z_),
        Vec3(params_.resample_var_roll_,
             params_.resample_var_pitch_,
             params_.resample_var_yaw_)));

    std::normal_distribution<float> noise(0.0, 1.0);
    auto update_noise_func = [this, &noise](State6DOF &s)
    {
      s.noise_ll_ = noise(engine_) * params_.odom_err_lin_lin_;
      s.noise_la_ = noise(engine_) * params_.odom_err_lin_ang_;
      s.noise_aa_ = noise(engine_) * params_.odom_err_ang_ang_;
      s.noise_al_ = noise(engine_) * params_.odom_err_ang_lin_;
    };
    pf_->predict(update_noise_func);

    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("MCL (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    const auto err_integ_map = e_max.rot * e_max.odom_err_integ_lin;
    ROS_DEBUG("odom error integral lin: %0.3f, %0.3f, %0.3f, "
              "ang: %0.3f, %0.3f, %0.3f, "
              "pos: %0.3f, %0.3f, %0.3f, "
              "err on map: %0.3f, %0.3f, %0.3f",
              e_max.odom_err_integ_lin.x,
              e_max.odom_err_integ_lin.y,
              e_max.odom_err_integ_lin.z,
              e_max.odom_err_integ_ang.x,
              e_max.odom_err_integ_ang.y,
              e_max.odom_err_integ_ang.z,
              e_max.pos.x,
              e_max.pos.y,
              e_max.pos.z,
              err_integ_map.x,
              err_integ_map.y,
              err_integ_map.z);
    ROS_DEBUG("match ratio min: %0.3f, max: %0.3f, pos: %0.3f, %0.3f, %0.3f",
              match_ratio_min,
              match_ratio_max,
              e.pos.x,
              e.pos.y,
              e.pos.z);
    if (match_ratio_max < params_.match_ratio_thresh_)
    {
      ROS_WARN_THROTTLE(3.0, "Low match_ratio. Expansion resetting.");
      pf_->noise(State6DOF(
          Vec3(params_.expansion_var_x_,
               params_.expansion_var_y_,
               params_.expansion_var_z_),
          Vec3(params_.expansion_var_roll_,
               params_.expansion_var_pitch_,
               params_.expansion_var_yaw_)));
      status.status = mcl_3dl_msgs::Status::EXPANSION_RESETTING;
    }
    pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_accum_header_.clear();

    ros::Time localized_current = ros::Time::now();
    float dt = (localized_current - localized_last_).toSec();
    if (dt > 1.0)
      dt = 1.0;
    else if (dt < 0.0)
      dt = 0.0;
    tf_tolerance_base_ = ros::Duration(localize_rate_->in(dt));
    localized_last_ = localized_current;

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      const int reduced = pf_->getParticleSize() * 0.75;
      if (reduced > params_.num_particles_)
      {
        pf_->resizeParticle(reduced);
      }
      else
      {
        pf_->resizeParticle(params_.num_particles_);
      }
      // wait 99.7% fix (three-sigma)
      global_localization_fix_cnt_ = 1 + ceil(params_.lpf_step_) * 3.0;
    }
    if (global_localization_fix_cnt_)
    {
      global_localization_fix_cnt_--;
      status.status = mcl_3dl_msgs::Status::GLOBAL_LOCALIZATION;
    }

    status.match_ratio = match_ratio_max;
    status.particle_size = pf_->getParticleSize();
    pub_status_.publish(status);
  }
  void cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    NormalLikelihoodNd<float, 6> nd(
        Eigen::Matrix<double, 6, 6>(
            msg->pose.covariance.data())
            .cast<float>());
    const State6DOF measured(
        Vec3(msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z),
        Quat(msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z,
             msg->pose.pose.orientation.w));
    auto measure_func = [this, &measured, &nd](const State6DOF &s) -> float
    {
      State6DOF diff = s - measured;
      const Vec3 rot_rpy = diff.rot.getRPY();
      const Eigen::Matrix<float, 6, 1> diff_vec =
          (Eigen::MatrixXf(6, 1) << diff.pos.x,
           diff.pos.y,
           diff.pos.z,
           rot_rpy.x,
           rot_rpy.y,
           rot_rpy.z)
              .finished();

      return nd(diff_vec);
    };
    pf_->measure(measure_func);
  }
  void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
  {
    Vec3 acc;
    acc.x = f_acc_[0]->in(msg->linear_acceleration.x);
    acc.y = f_acc_[1]->in(msg->linear_acceleration.y);
    acc.z = f_acc_[2]->in(msg->linear_acceleration.z);

    if (!has_imu_)
    {
      f_acc_[0]->set(0.0);
      f_acc_[1]->set(0.0);
      f_acc_[2]->set(0.0);
      imu_last_ = msg->header.stamp;
      has_imu_ = true;
      return;
    }

    float dt = (msg->header.stamp - imu_last_).toSec();
    if (dt < 0.0 || dt > 5.0)
    {
      ROS_WARN("Detected time jump in imu. Resetting.");
      has_imu_ = false;
      return;
    }
    else if (dt > 0.05)
    {
      Vec3 acc_measure = acc / acc.norm();
      try
      {
        tf::Stamped<tf::Vector3> in, out;
        in.frame_id_ = msg->header.frame_id;
        in.stamp_ = msg->header.stamp;
        in.setX(acc_measure.x);
        in.setY(acc_measure.y);
        in.setZ(acc_measure.z);
        tfl_.transformVector(frame_ids_["base_link"], in, out);
        acc_measure = Vec3(out.x(), out.y(), out.z());

        tf::StampedTransform trans;
        // Static transform
        tfl_.lookupTransform(frame_ids_["base_link"], msg->header.frame_id, ros::Time(0), trans);

        imu_quat_.x = msg->orientation.x;
        imu_quat_.y = msg->orientation.y;
        imu_quat_.z = msg->orientation.z;
        imu_quat_.w = msg->orientation.w;
        Vec3 axis;
        float angle;
        imu_quat_.getAxisAng(axis, angle);
        axis = Quat(trans.getRotation().x(),
                    trans.getRotation().y(),
                    trans.getRotation().z(),
                    trans.getRotation().w()) *
               axis;
        imu_quat_.setAxisAng(axis, angle);
      }
      catch (tf::TransformException &e)
      {
        return;
      }
      const float acc_measure_norm = acc_measure.norm();
      NormalLikelihood<float> nd(params_.acc_var_);
      auto imu_measure_func = [this, &nd, &acc_measure, &acc_measure_norm](const State6DOF &s) -> float
      {
        const Vec3 acc_estim = s.rot.inv() * Vec3(0.0, 0.0, 1.0);
        const float diff = acosf(
            acc_estim.dot(acc_measure) / (acc_measure_norm * acc_estim.norm()));
        return nd(diff);
      };
      pf_->measure(imu_measure_func);

      imu_last_ = msg->header.stamp;
    }
  }
  bool cbResizeParticle(mcl_3dl_msgs::ResizeParticleRequest &request,
                        mcl_3dl_msgs::ResizeParticleResponse &response)
  {
    pf_->resizeParticle(request.size);
    return true;
  }
  bool cbExpansionReset(std_srvs::TriggerRequest &request,
                        std_srvs::TriggerResponse &response)
  {
    pf_->noise(State6DOF(
        Vec3(params_.expansion_var_x_,
             params_.expansion_var_y_,
             params_.expansion_var_z_),
        Vec3(params_.expansion_var_roll_,
             params_.expansion_var_pitch_,
             params_.expansion_var_yaw_)));
    return true;
  }
  bool cbGlobalLocalization(std_srvs::TriggerRequest &request,
                            std_srvs::TriggerResponse &response)
  {
    if (!has_map_)
    {
      response.success = false;
      response.message = "No map received.";
      return true;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_map_);
    ds.setLeafSize(
        params_.global_localization_grid_,
        params_.global_localization_grid_,
        params_.global_localization_grid_);
    ds.filter(*points);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree->setInputCloud(points);

    auto pc_filter = [this, kdtree](const pcl::PointXYZI &p)
    {
      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      auto p2 = p;
      p2.z += 0.01 + params_.global_localization_grid_;

      return kdtree->radiusSearch(
          p2, params_.global_localization_grid_, id, sqdist, 1);
    };
    points->erase(
        std::remove_if(points->begin(), points->end(), pc_filter),
        points->end());

    const int dir = params_.global_localization_div_yaw_;
    pf_->resizeParticle(points->size() * dir);
    auto pit = points->begin();

    const float prob = 1.0 / static_cast<float>(points->size());
    int cnt = 0;
    for (auto &particle : *pf_)
    {
      assert(pit != points->end());
      particle.probability = prob;
      particle.probability_bias = 1.0;
      particle.state.pos.x = pit->x;
      particle.state.pos.y = pit->y;
      particle.state.pos.z = pit->z;
      particle.state.rot = Quat(Vec3(0.0, 0.0, 2.0 * M_PI * cnt / dir)) * imu_quat_;
      particle.state.rot.normalize();
      if (++cnt >= dir)
      {
        cnt = 0;
        ++pit;
      }
    }
    response.success = true;
    response.message = std::to_string(points->size()) + " particles";
    return true;
  }

public:
  MCL3dlNode(int argc, char *argv[])
    : nh_("")
    , pnh_("~")
    , global_localization_fix_cnt_(0)
    , engine_(seed_gen_())
  {
    mcl_3dl_compat::checkCompatMode();
    sub_cloud_ = mcl_3dl_compat::subscribe(
        nh_, "cloud",
        pnh_, "cloud", 100, &MCL3dlNode::cbCloud, this);
    sub_odom_ = mcl_3dl_compat::subscribe(
        nh_, "odom",
        pnh_, "odom", 200, &MCL3dlNode::cbOdom, this);
    sub_imu_ = mcl_3dl_compat::subscribe(
        nh_, "imu/data",
        pnh_, "imu", 200, &MCL3dlNode::cbImu, this);
    sub_mapcloud_ = mcl_3dl_compat::subscribe(
        nh_, "mapcloud",
        pnh_, "mapcloud", 1, &MCL3dlNode::cbMapcloud, this);
    sub_mapcloud_update_ = mcl_3dl_compat::subscribe(
        nh_, "mapcloud_update",
        pnh_, "mapcloud_update", 1, &MCL3dlNode::cbMapcloudUpdate, this);
    sub_position_ = mcl_3dl_compat::subscribe(
        nh_, "initialpose",
        pnh_, "initialpose", 1, &MCL3dlNode::cbPosition, this);
    sub_landmark_ = mcl_3dl_compat::subscribe(
        nh_, "mcl_measurement",
        pnh_, "landmark", 1, &MCL3dlNode::cbLandmark, this);

    pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, false);
    pub_particle_ = pnh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
    pub_mapcloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("updated_map", 1, true);
    pub_debug_marker_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug_marker", 1, true);
    pub_status_ = pnh_.advertise<mcl_3dl_msgs::Status>("status", 1, true);
    pub_matched_ = pnh_.advertise<sensor_msgs::PointCloud2>("matched", 2, true);
    pub_unmatched_ = pnh_.advertise<sensor_msgs::PointCloud2>("unmatched", 2, true);

    srv_particle_size_ = mcl_3dl_compat::advertiseService(
        nh_, "resize_mcl_particle",
        pnh_, "resize_particle", &MCL3dlNode::cbResizeParticle, this);
    srv_global_localization_ = mcl_3dl_compat::advertiseService(
        nh_, "global_localization",
        pnh_, "global_localization", &MCL3dlNode::cbGlobalLocalization, this);
    srv_expansion_reset_ = mcl_3dl_compat::advertiseService(
        nh_, "expansion_resetting",
        pnh_, "expansion_resetting", &MCL3dlNode::cbExpansionReset, this);

    pnh_.param("map_frame", frame_ids_["map"], std::string("map"));
    pnh_.param("robot_frame", frame_ids_["base_link"], std::string("base_link"));
    pnh_.param("odom_frame", frame_ids_["odom"], std::string("odom"));
    pnh_.param("floor_frame", frame_ids_["floor"], std::string("floor"));

    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/clip_near", "clip_near");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/clip_far", "clip_far");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/clip_z_min", "clip_z_min");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/clip_z_max", "clip_z_max");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/match_dist_min", "match_dist_min");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/match_dist_flat", "match_dist_flat");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/match_weight", "match_weight");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/num_points", "num_points");
    mcl_3dl_compat::paramRename<double>(pnh_, "likelihood/num_points_global", "num_points_global");

    mcl_3dl_compat::paramRename<double>(pnh_, "beam/clip_near", "clip_beam_near");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/clip_far", "clip_beam_far");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/clip_z_min", "clip_beam_z_min");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/clip_z_max", "clip_beam_z_max");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/num_points", "num_points_beam");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/beam_likelihood", "beam_likelihood");
    mcl_3dl_compat::paramRename<double>(pnh_, "beam/ang_total_ref", "ang_total_ref");

    pnh_.param("map_downsample_x", params_.map_downsample_x_, 0.1);
    pnh_.param("map_downsample_y", params_.map_downsample_y_, 0.1);
    pnh_.param("map_downsample_z", params_.map_downsample_z_, 0.1);
    pnh_.param("downsample_x", params_.downsample_x_, 0.1);
    pnh_.param("downsample_y", params_.downsample_y_, 0.1);
    pnh_.param("downsample_z", params_.downsample_z_, 0.05);
    params_.map_grid_min_ = std::min(std::min(params_.map_downsample_x_, params_.map_downsample_y_),
                                     params_.map_downsample_z_);
    params_.map_grid_max_ = std::max(std::max(params_.map_downsample_x_, params_.map_downsample_y_),
                                     params_.map_downsample_z_);
    pnh_.param("update_downsample_x", params_.update_downsample_x_, 0.3);
    pnh_.param("update_downsample_y", params_.update_downsample_y_, 0.3);
    pnh_.param("update_downsample_z", params_.update_downsample_z_, 0.3);
    double map_update_interval_t;
    pnh_.param("map_update_interval_interval", map_update_interval_t, 2.0);
    params_.map_update_interval_.reset(new ros::Duration(map_update_interval_t));

    double weight[3];
    float weight_f[4];
    pnh_.param("dist_weight_x", weight[0], 1.0);
    pnh_.param("dist_weight_y", weight[1], 1.0);
    pnh_.param("dist_weight_z", weight[2], 5.0);
    for (size_t i = 0; i < 3; i++)
      weight_f[i] = weight[i];
    weight_f[3] = 0.0;
    point_rep_.setRescaleValues(weight_f);

    pnh_.param("global_localization_grid_lin", params_.global_localization_grid_, 0.3);
    double grid_ang;
    pnh_.param("global_localization_grid_ang", grid_ang, 0.524);
    params_.global_localization_div_yaw_ = lroundf(2 * M_PI / grid_ang);

    pnh_.param("num_particles", params_.num_particles_, 64);
    pf_.reset(new pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat>(params_.num_particles_));

    pnh_.param("resample_var_x", params_.resample_var_x_, 0.05);
    pnh_.param("resample_var_y", params_.resample_var_y_, 0.05);
    pnh_.param("resample_var_z", params_.resample_var_z_, 0.05);
    pnh_.param("resample_var_roll", params_.resample_var_roll_, 0.05);
    pnh_.param("resample_var_pitch", params_.resample_var_pitch_, 0.05);
    pnh_.param("resample_var_yaw", params_.resample_var_yaw_, 0.05);
    pnh_.param("expansion_var_x", params_.expansion_var_x_, 0.2);
    pnh_.param("expansion_var_y", params_.expansion_var_y_, 0.2);
    pnh_.param("expansion_var_z", params_.expansion_var_z_, 0.2);
    pnh_.param("expansion_var_roll", params_.expansion_var_roll_, 0.05);
    pnh_.param("expansion_var_pitch", params_.expansion_var_pitch_, 0.05);
    pnh_.param("expansion_var_yaw", params_.expansion_var_yaw_, 0.05);
    pnh_.param("match_ratio_thresh", params_.match_ratio_thresh_, 0.0);

    pnh_.param("odom_err_lin_lin", params_.odom_err_lin_lin_, 0.10);
    pnh_.param("odom_err_lin_ang", params_.odom_err_lin_ang_, 0.05);
    pnh_.param("odom_err_ang_lin", params_.odom_err_ang_lin_, 0.05);
    pnh_.param("odom_err_ang_ang", params_.odom_err_ang_ang_, 0.05);

    pnh_.param("odom_err_integ_lin_tc", params_.odom_err_integ_lin_tc_, 10.0);
    pnh_.param("odom_err_integ_lin_sigma", params_.odom_err_integ_lin_sigma_, 100.0);
    pnh_.param("odom_err_integ_ang_tc", params_.odom_err_integ_ang_tc_, 10.0);
    pnh_.param("odom_err_integ_ang_sigma", params_.odom_err_integ_ang_sigma_, 100.0);

    double x, y, z;
    double roll, pitch, yaw;
    double v_x, v_y, v_z;
    double v_roll, v_pitch, v_yaw;
    pnh_.param("init_x", x, 0.0);
    pnh_.param("init_y", y, 0.0);
    pnh_.param("init_z", z, 0.0);
    pnh_.param("init_roll", roll, 0.0);
    pnh_.param("init_pitch", pitch, 0.0);
    pnh_.param("init_yaw", yaw, 0.0);
    pnh_.param("init_var_x", v_x, 2.0);
    pnh_.param("init_var_y", v_y, 2.0);
    pnh_.param("init_var_z", v_z, 0.5);
    pnh_.param("init_var_roll", v_roll, 0.1);
    pnh_.param("init_var_pitch", v_pitch, 0.1);
    pnh_.param("init_var_yaw", v_yaw, 0.5);
    pf_->init(
        State6DOF(
            Vec3(x, y, z),
            Quat(Vec3(roll, pitch, yaw))),
        State6DOF(
            Vec3(v_x, v_y, v_z),
            Vec3(v_roll, v_pitch, v_yaw)));

    pnh_.param("lpf_step", params_.lpf_step_, 16.0);
    f_pos_[0].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0));
    f_pos_[1].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0));
    f_pos_[2].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0));
    f_ang_[0].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0, true));
    f_ang_[1].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0, true));
    f_ang_[2].reset(new Filter(Filter::FILTER_LPF, params_.lpf_step_, 0.0, true));

    double acc_lpf_step;
    pnh_.param("acc_lpf_step", acc_lpf_step, 128.0);
    f_acc_[0].reset(new Filter(Filter::FILTER_LPF, acc_lpf_step, 0.0));
    f_acc_[1].reset(new Filter(Filter::FILTER_LPF, acc_lpf_step, 0.0));
    f_acc_[2].reset(new Filter(Filter::FILTER_LPF, acc_lpf_step, 0.0));
    pnh_.param("acc_var", params_.acc_var_, M_PI / 4.0);  // 45 deg

    pnh_.param("jump_dist", params_.jump_dist_, 1.0);
    pnh_.param("jump_ang", params_.jump_ang_, 1.57);
    pnh_.param("fix_dist", params_.fix_dist_, 0.2);
    pnh_.param("fix_ang", params_.fix_ang_, 0.1);
    pnh_.param("bias_var_dist", params_.bias_var_dist_, 2.0);
    pnh_.param("bias_var_ang", params_.bias_var_ang_, 1.57);

    pnh_.param("skip_measure", params_.skip_measure_, 1);
    cnt_measure_ = 0;
    pnh_.param("accum_cloud", params_.accum_cloud_, 1);
    cnt_accum_ = 0;

    pnh_.param("match_output_dist", params_.match_output_dist_, 0.1);
    pnh_.param("unmatch_output_dist", params_.unmatch_output_dist_, 0.5);
    double match_output_interval_t;
    pnh_.param("match_output_interval_interval", match_output_interval_t, 0.2);
    params_.match_output_interval_.reset(new ros::Duration(match_output_interval_t));

    double tf_tolerance_t;
    pnh_.param("tf_tolerance", tf_tolerance_t, 0.05);
    params_.tf_tolerance_.reset(new ros::Duration(tf_tolerance_t));

    pnh_.param("publish_tf", publish_tf_, true);
    pnh_.param("output_pcd", output_pcd_, false);

    imu_quat_ = Quat(0.0, 0.0, 0.0, 1.0);

    has_odom_ = has_map_ = has_imu_ = false;
    localize_rate_.reset(new Filter(Filter::FILTER_LPF, 5.0, 0.0));

    lidar_measurements_["likelihood"] =
        LidarMeasurementModelBase::Ptr(
            new LidarMeasurementModelLikelihood());
    lidar_measurements_["beam"] =
        LidarMeasurementModelBase::Ptr(
            new LidarMeasurementModelBeam(
                params_.map_downsample_x_, params_.map_downsample_y_, params_.map_downsample_z_));

    float max_search_radius = 0;
    for (auto &lm : lidar_measurements_)
    {
      lm.second->loadConfig(pnh_, lm.first);
      max_search_radius = std::max(max_search_radius, lm.second->getMaxSearchRange());
    }

    double map_chunk;
    pnh_.param("map_chunk", map_chunk, 20.0);
    ROS_DEBUG("max_search_radius: %0.3f", max_search_radius);
    kdtree_.reset(new ChunkedKdtree<pcl::PointXYZI>(map_chunk, max_search_radius));
    kdtree_->setEpsilon(params_.map_grid_min_ / 16);
    kdtree_->setPointRepresentation(
        boost::dynamic_pointer_cast<
            pcl::PointRepresentation<pcl::PointXYZI>,
            MyPointRepresentation>(boost::make_shared<MyPointRepresentation>(point_rep_)));

    map_update_timer_ = nh_.createTimer(
        *params_.map_update_interval_,
        &MCL3dlNode::cbMapUpdateTimer, this);
  }
  ~MCL3dlNode()
  {
    if (output_pcd_ && pc_all_accum_)
    {
      std::cerr << "mcl_3dl: saving pcd file.";
      std::cerr << " (" << pc_all_accum_->points.size() << " points)" << std::endl;
      pcl::io::savePCDFileBinary("mcl_3dl.pcd", *pc_all_accum_);
    }
  }

  void cbMapUpdateTimer(const ros::TimerEvent &event)
  {
    if (has_map_)
    {
      const auto ts = boost::chrono::high_resolution_clock::now();
      if (pc_update_)
      {
        if (!pc_map2_)
          pc_map2_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        *pc_map2_ = *pc_map_ + *pc_update_;
        pc_update_.reset();
      }
      else
      {
        if (pc_map2_)
          return;
        pc_map2_ = pc_map_;
      }
      kdtree_->setInputCloud(pc_map2_);

      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*pc_map2_, out);
      pub_mapcloud_.publish(out);
      const auto tnow = boost::chrono::high_resolution_clock::now();
      ROS_DEBUG("Map update (%0.3f sec.)",
                boost::chrono::duration<float>(tnow - ts).count());
    }
  }

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

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  std::shared_ptr<Filter> f_pos_[3];
  std::shared_ptr<Filter> f_ang_[3];
  std::shared_ptr<Filter> f_acc_[3];
  std::shared_ptr<Filter> localize_rate_;
  ros::Time localized_last_;
  ros::Duration tf_tolerance_base_;

  Parameters params_;
  std::map<std::string, std::string> frame_ids_;
  bool output_pcd_;
  bool publish_tf_;

  ros::Time match_output_last_;
  ros::Time odom_last_;
  bool has_map_;
  bool has_odom_;
  bool has_imu_;
  State6DOF odom_;
  State6DOF odom_prev_;
  std::map<std::string, bool> frames_;
  std::vector<std::string> frames_v_;
  size_t frame_num_;
  State6DOF state_prev_;
  ros::Time imu_last_;
  int cnt_measure_;
  int cnt_accum_;
  Quat imu_quat_;
  size_t global_localization_fix_cnt_;

  MyPointRepresentation point_rep_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map2_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_update_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_all_accum_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_accum_;
  ChunkedKdtree<pcl::PointXYZI>::Ptr kdtree_;
  std::vector<std_msgs::Header> pc_accum_header_;

  std::map<
      std::string,
      LidarMeasurementModelBase::Ptr> lidar_measurements_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl_3dl

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mcl_3dl");

  mcl_3dl::MCL3dlNode mcl(argc, argv);
  ros::spin();

  return 0;
}

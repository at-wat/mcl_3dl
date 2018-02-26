/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <mcl_3dl/ResizeParticle.h>
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

#include <boost/chrono.hpp>
#include <algorithm>
#include <string>
#include <map>
#include <vector>

#include <pf.h>
#include <vec3.h>
#include <quat.h>
#include <filter.h>
#include <nd.h>
#include <raycast.h>

class MCL3dlNode
{
protected:
  class Parameters
  {
  public:
    double clip_near;
    double clip_far;
    double clip_near_sq;
    double clip_far_sq;
    double clip_z_min;
    double clip_z_max;
    double clip_beam_near;
    double clip_beam_far;
    double clip_beam_near_sq;
    double clip_beam_far_sq;
    double clip_beam_z_min;
    double clip_beam_z_max;
    double map_clip_z_min;
    double map_clip_z_max;
    double map_clip_far;
    double map_clip_far_sq;
    double map_downsample_x;
    double map_downsample_y;
    double map_downsample_z;
    double update_downsample_x;
    double update_downsample_y;
    double update_downsample_z;
    double map_grid_min;
    double global_localization_grid;
    int global_localization_div_yaw;
    double downsample_x;
    double downsample_y;
    double downsample_z;
    double resample_var_x;
    double resample_var_y;
    double resample_var_z;
    double resample_var_roll;
    double resample_var_pitch;
    double resample_var_yaw;
    double match_dist_min;
    double match_weight;
    double jump_dist;
    double jump_ang;
    double fix_dist;
    double fix_ang;
    double odom_err_lin_lin;
    double odom_err_lin_ang;
    double odom_err_ang_lin;
    double odom_err_ang_ang;
    std::shared_ptr<ros::Duration> map_update_interval;
    int num_particles;
    int num_points;
    int num_points_global;
    int num_points_beam;
    int skip_measure;
    int accum_cloud;
    double match_output_dist;
    double unmatch_output_dist;
    double bias_var_dist;
    double bias_var_ang;
    float beam_likelihood;
    double beam_likelihood_min;
    float sin_total_ref;
    double acc_var;
    double odom_err_integ_tc;
    double odom_err_integ_sigma;
    std::shared_ptr<ros::Duration> match_output_interval;
    std::shared_ptr<ros::Duration> tf_tolerance;
  };

  class State : public pf::ParticleBase<float>
  {
  public:
    Vec3 pos;
    Quat rot;
    bool diff;
    float noise_ll_;
    float noise_la_;
    float noise_al_;
    float noise_aa_;
    Vec3 odom_err_integ;
    class RPYVec
    {
    public:
      Vec3 v;
      RPYVec()
      {
      }
      explicit RPYVec(const Vec3 &v)
      {
        this->v = v;
      }
      RPYVec(const float &r, const float &p, const float y)
      {
        this->v.x = r;
        this->v.y = p;
        this->v.z = y;
      }
    };
    RPYVec rpy;
    float &operator[](const size_t i)override
    {
      switch (i)
      {
        case 0:
          return pos.x;
        case 1:
          return pos.y;
        case 2:
          return pos.z;
        case 3:
          return rot.x;
        case 4:
          return rot.y;
        case 5:
          return rot.z;
        case 6:
          return rot.w;
        case 7:
          return odom_err_integ.x;
        case 8:
          return odom_err_integ.y;
        case 9:
          return odom_err_integ.z;
      }
      return pos.x;
    }
    size_t size() const override
    {
      return 10;
    }
    void normalize() override
    {
      rot.normalize();
    }
    State()
    {
      diff = false;
      noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
      odom_err_integ = Vec3(0.0, 0.0, 0.0);
    }
    State(const Vec3 pos, const Quat rot)
    {
      this->pos = pos;
      this->rot = rot;
      noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
      odom_err_integ = Vec3(0.0, 0.0, 0.0);
      diff = false;
    }
    State(const Vec3 pos, const Vec3 rpy)
    {
      this->pos = pos;
      this->rpy = RPYVec(rpy);
      noise_ll_ = noise_la_ = noise_aa_ = noise_al_ = 0.0;
      odom_err_integ = Vec3(0.0, 0.0, 0.0);
      diff = true;
    }
    bool isDiff()
    {
      return diff;
    }
    void transform(pcl::PointCloud<pcl::PointXYZI> &pc) const
    {
      auto r = rot.normalized();
      for (auto &p : pc.points)
      {
        auto t = r * Vec3(p.x, p.y, p.z) + pos;
        p.x = t.x;
        p.y = t.y;
        p.z = t.z;
      }
    }
    static State generateNoise(
        std::default_random_engine &engine_,
        State mean, State sigma)
    {
      State noise;
      if (mean.isDiff() || !sigma.isDiff())
      {
        ROS_ERROR("Failed to generate noise. mean must be Quat and sigma must be rpy vec.");
      }
      for (size_t i = 0; i < 3; i++)
      {
        std::normal_distribution<float> nd(mean[i], sigma[i]);
        noise[i] = noise[i + 7] = nd(engine_);
      }
      Vec3 rpy_noise;
      for (size_t i = 0; i < 3; i++)
      {
        std::normal_distribution<float> nd(0.0, sigma.rpy.v[i]);
        rpy_noise[i] = nd(engine_);
      }
      noise.rot = Quat(rpy_noise) * mean.rot;
      return noise;
    }
    State operator+(const State &a)
    {
      State in = a;
      State ret;
      for (size_t i = 0; i < size(); i++)
      {
        if (3 <= i && i <= 6)
          continue;
        ret[i] = (*this)[i] + in[i];
      }
      ret.rot = a.rot * rot;
      return ret;
    }
  };
  class ParticleWeightedMeanQuat : public pf::ParticleWeightedMean<State, float>
  {
  protected:
    Vec3 front_sum_;
    Vec3 up_sum_;

  public:
    ParticleWeightedMeanQuat()
      : ParticleWeightedMean()
      , front_sum_(0.0, 0.0, 0.0)
      , up_sum_(0.0, 0.0, 0.0)
    {
    }

    void add(const State &s, const float &prob)
    {
      p_sum_ += prob;

      State e1 = s;
      e_.pos += e1.pos * prob;

      const Vec3 front = s.rot * Vec3(1.0, 0.0, 0.0) * prob;
      const Vec3 up = s.rot * Vec3(0.0, 0.0, 1.0) * prob;

      front_sum_ += front;
      up_sum_ += up;
    }

    State getMean()
    {
      assert(p_sum_ > 0.0);

      return State(e_.pos / p_sum_, Quat(front_sum_, up_sum_));
    }

    float getTotalProbability()
    {
      return p_sum_;
    }
  };
  std::shared_ptr<pf::ParticleFilter<State, float, ParticleWeightedMeanQuat>> pf_;

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
    pc_map2_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_update_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_tmp.makeShared());
    ds.setLeafSize(params_.map_downsample_x, params_.map_downsample_y, params_.map_downsample_z);
    ds.filter(*pc_map_);
    pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_all_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    frame_num_ = 0;
    has_map_ = true;

    kdtree_orig_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_orig_->setInputCloud(pc_map_);
    std::vector<int> id(7);
    std::vector<float> sqdist(7);
    ROS_INFO("map original: %d points", (int)pc_map_->points.size());

    // map pointcloud filters here

    pc_map_->width = 1;
    pc_map_->height = pc_map_->points.size();
    ROS_INFO("map reduced: %d points", (int)pc_map_->points.size());
    kdtree_orig_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_orig_->setInputCloud(pc_map_);

    *pc_map2_ = *pc_map_;
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_->setEpsilon(params_.map_grid_min / 4);
    kdtree_->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep_));

    if (pc_map2_->points.size() == 0)
      kdtree_->setInputCloud(pc_map_);
    else
      kdtree_->setInputCloud(pc_map2_);
  }
  void cbMapcloudUpdate(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    ROS_INFO("map_update received");
    pcl::PointCloud<pcl::PointXYZI> pc_tmp;
    pcl::fromROSMsg(*msg, pc_tmp);

    pc_update_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_tmp.makeShared());
    ds.setLeafSize(params_.update_downsample_x, params_.update_downsample_y, params_.update_downsample_z);
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
        State(
            Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            Quat(pose.pose.orientation.x,
                 pose.pose.orientation.y,
                 pose.pose.orientation.z,
                 pose.pose.orientation.w)),
        State(
            Vec3(msg->pose.covariance[0],
                 msg->pose.covariance[6 * 1 + 1],
                 msg->pose.covariance[6 * 2 + 2]),
            Vec3(msg->pose.covariance[6 * 3 + 3],
                 msg->pose.covariance[6 * 4 + 4],
                 msg->pose.covariance[6 * 5 + 5])));
    pc_update_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    auto integ_reset_func = [](State &s)
    {
      s.odom_err_integ = Vec3();
    };
    pf_->predict(integ_reset_func);
  }

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_ =
        State(
            Vec3(msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z),
            Quat(msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z,
                 msg->pose.pose.orientation.w));
    if (has_odom_)
    {
      const float dt = (msg->header.stamp - odom_last_).toSec();
      if (dt < 0.0)
      {
        has_odom_ = false;
      }
      else if (dt > 0.05)
      {
        const Vec3 v = odom_prev_.rot.inv() * (odom_.pos - odom_prev_.pos);
        const Quat r = odom_prev_.rot.inv() * odom_.rot;
        Vec3 axis;
        float ang;
        r.getAxisAng(axis, ang);

        const float trans = v.norm();
        auto prediction_func = [this, &v, &r, axis, ang, trans, &dt](State &s)
        {
          const Vec3 diff = v * (1.0 + s.noise_ll_) + Vec3(s.noise_al_ * ang, 0.0, 0.0);
          s.odom_err_integ += (diff - v);
          s.pos += s.rot * diff;
          s.rot = Quat(Vec3(0.0, 0.0, 1.0), s.noise_la_ * trans + s.noise_aa_ * ang) *
                  s.rot * r;
          s.rot.normalize();
          s.odom_err_integ *= (1.0 - dt / params_.odom_err_integ_tc);
        };
        pf_->predict(prediction_func);
        odom_last_ = msg->header.stamp;
        odom_prev_ = odom_;
      }
    }
    else
    {
      odom_prev_ = odom_;
      odom_last_ = msg->header.stamp;
      has_odom_ = true;
    }
  }
  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    if (!has_map_)
      return;
    if (frames_.find(msg->header.frame_id) == frames_.end())
    {
      frames_[msg->header.frame_id] = true;
      frames_v_.push_back(msg->header.frame_id);
    }

    sensor_msgs::PointCloud2 pc_bl;
    if (!pcl_ros::transformPointCloud(frame_ids_["odom"], *msg, pc_bl, tfl_))
    {
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
    if (cnt_accum_ % params_.accum_cloud != 0)
      return;

    cnt_measure_++;
    if (cnt_measure_ % params_.skip_measure != 0)
    {
      pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      pc_accum_header_.clear();
      return;
    }

    if (!pcl_ros::transformPointCloud(
            frame_ids_["base_link"], *pc_local_accum_, *pc_local_accum_, tfl_))
    {
      ROS_ERROR("Failed to transform laser origin.");
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
        ROS_ERROR("Failed to transform laser origin.");
        origins.push_back(Vec3(0.0, 0.0, 0.0));
      }
    }

    const auto ts = boost::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> ds;
    ds.setInputCloud(pc_local_accum_);
    ds.setLeafSize(params_.downsample_x, params_.downsample_y, params_.downsample_z);
    ds.filter(*pc_local);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_full(new pcl::PointCloud<pcl::PointXYZI>);
    *pc_local_full = *pc_local;

    std::uniform_real_distribution<float> ud(0.0, 1.0);
    int num_valid = 0;
    for (auto &p : pc_local->points)
    {
      if (p.x * p.x + p.y * p.y > params_.clip_far_sq)
        continue;
      if (p.x * p.x + p.y * p.y < params_.clip_near_sq)
        continue;
      if (p.z < params_.clip_z_min || params_.clip_z_max < p.z)
        continue;
      num_valid++;
    }
    auto random_sample = [this](
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc, const size_t &num)
        -> pcl::PointCloud<pcl::PointXYZI>::Ptr
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
      std::uniform_int_distribution<size_t> ud(0, pc->points.size() - 1);

      for (size_t i = 0; i < num; i++)
      {
        output->points.push_back(pc->points[ud(engine_)]);
      }
      output->width = 1;
      output->height = output->points.size();
      output->header.frame_id = pc->header.frame_id;
      output->header.stamp = pc->header.stamp;

      return output;
    };
    auto local_points_filter = [this](const pcl::PointXYZI &p)
    {
      if (p.x * p.x + p.y * p.y > params_.clip_far_sq)
        return true;
      if (p.x * p.x + p.y * p.y < params_.clip_near_sq)
        return true;
      if (p.z < params_.clip_z_min || params_.clip_z_max < p.z)
        return true;
      return false;
    };
    pc_local->erase(
        std::remove_if(pc_local->begin(), pc_local->end(), local_points_filter),
        pc_local->end());
    if (pc_local->size() == 0)
    {
      ROS_ERROR("All points are filtered out. Failed to localize.");
      return;
    }
    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
    {
      size_t num = params_.num_points;
      if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
        num = num * params_.num_particles / pf_->getParticleSize();
      if (static_cast<int>(num) < params_.num_points_global)
        num = params_.num_points_global;
      pc_local = random_sample(pc_local, static_cast<size_t>(num));
    }
    else
    {
      pc_local = random_sample(pc_local, static_cast<size_t>(params_.num_points));
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_beam(new pcl::PointCloud<pcl::PointXYZI>);
    *pc_local_beam = *pc_local;
    auto local_beam_filter = [this](const pcl::PointXYZI &p)
    {
      if (p.x * p.x + p.y * p.y > params_.clip_beam_far_sq)
        return true;
      if (p.x * p.x + p.y * p.y < params_.clip_beam_near_sq)
        return true;
      if (p.z < params_.clip_beam_z_min || params_.clip_beam_z_max < p.z)
        return true;
      if (p.intensity - roundf(p.intensity) > 0.01)
        return true;
      return false;
    };
    pc_local_beam->erase(
        std::remove_if(pc_local_beam->begin(), pc_local_beam->end(), local_beam_filter),
        pc_local_beam->end());
    if (pc_local_beam->size() == 0)
    {
      ROS_ERROR("All beam points are filtered out. Skipping beam model.");
    }
    else
    {
      size_t num = params_.num_points_beam;
      if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
        num = num * params_.num_particles / pf_->getParticleSize();
      pc_local_beam = random_sample(pc_local_beam, num);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle_beam(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_dummy(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<int> id(1);
    std::vector<float> sqdist(1);

    const float match_dist_min = params_.match_dist_min;
    const float match_weight = params_.match_weight;
    NormalLikelihood<float> odom_error_nd(params_.odom_err_integ_sigma);
    auto measure_func = [this, &match_dist_min, &match_weight,
                         &id, &sqdist, &pc_particle, &pc_local,
                         &pc_particle_beam, &pc_local_beam, &origins,
                         &odom_error_nd](const State &s) -> float
    {
      // likelihood model
      float score_like = 0;
      *pc_particle = *pc_local;
      s.transform(*pc_particle);
      size_t num = 0;
      for (auto &p : pc_particle->points)
      {
        if (kdtree_->radiusSearch(p, match_dist_min, id, sqdist, 1))
        {
          float dist = sqdist[0];
          if (dist < 0.05 * 0.05)
            dist = 0.05 * 0.05;
          dist = match_dist_min - sqrtf(dist);
          if (dist < 0.0)
            continue;

          score_like += dist * match_weight;
          num++;
        }
      }

      // approximative beam model
      float score_beam = 1.0;
      *pc_particle_beam = *pc_local_beam;
      s.transform(*pc_particle_beam);
      for (auto &p : pc_particle_beam->points)
      {
        const int beam_header_id = lroundf(p.intensity);
        Raycast ray(
            kdtree_,
            s.pos + s.rot * origins[beam_header_id],
            Vec3(p.x, p.y, p.z),
            params_.map_grid_min);
        for (auto point : ray)
        {
          if (point.collision_)
          {
            // reject total reflection
            if (point.sin_angle_ > params_.sin_total_ref)
            {
              score_beam *= params_.beam_likelihood;
            }
            break;
          }
        }
      }
      if (score_beam < params_.beam_likelihood_min)
        score_beam = params_.beam_likelihood_min;

      // odometry error integration
      const float odom_error =
          odom_error_nd(s.odom_err_integ.norm());
      return score_like * score_beam * odom_error;
    };
    pf_->measure(measure_func);

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
    {
      auto bias_func = [](const State &s, float &p_bias) -> void
      {
        p_bias = 1.0;
      };
      pf_->bias(bias_func);
    }
    else
    {
      NormalLikelihood<float> nl_lin(params_.bias_var_dist);
      NormalLikelihood<float> nl_ang(params_.bias_var_ang);
      auto bias_func = [this, &nl_lin, &nl_ang](const State &s, float &p_bias) -> void
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

    assert(std::isfinite(e.pos.x));
    assert(std::isfinite(e.pos.y));
    assert(std::isfinite(e.pos.z));
    assert(std::isfinite(e.rot.x));
    assert(std::isfinite(e.rot.y));
    assert(std::isfinite(e.rot.z));
    assert(std::isfinite(e.rot.w));

    e.rot.normalize();

    {
      visualization_msgs::MarkerArray markers;

      *pc_particle_beam = *pc_local_beam;
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
      for (auto &p : pc_particle_beam->points)
      {
        const int beam_header_id = lroundf(p.intensity);
        Raycast ray(
            kdtree_,
            e.pos + e.rot * origins[beam_header_id],
            Vec3(p.x, p.y, p.z),
            params_.map_grid_min);
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
            if (point.sin_angle_ > params_.sin_total_ref)
            {
              marker.color.a = 0.2;
            }
            markers.markers.push_back(marker);
            break;
          }
        }
      }

      *pc_particle = *pc_local;
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
    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
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
      if (jump_dist > params_.jump_dist ||
          fabs(jump_ang) > params_.jump_ang)
      {
        ROS_INFO("Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
        jump = true;

        auto integ_reset_func = [](State &s)
        {
          s.odom_err_integ = Vec3();
        };
        pf_->predict(integ_reset_func);
      }
      state_prev_ = e;
    }
    tf::StampedTransform trans;
    trans.stamp_ = odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance;
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
      if (fix_dist < params_.fix_dist &&
          fabs(fix_ang) < params_.fix_ang)
      {
        fix = true;
      }

      if (fix)
        ROS_DEBUG("Localization fixed");
    }
    *pc_particle = *pc_local;
    e.transform(*pc_particle);
    sensor_msgs::PointCloud pc;
    pc.header.stamp = msg->header.stamp;
    pc.header.frame_id = frame_ids_["map"];
    for (auto &p : pc_particle->points)
    {
      geometry_msgs::Point32 pp;
      pp.x = p.x;
      pp.y = p.y;
      pp.z = p.z;

      if (kdtree_orig_->nearestKSearch(p, 1, id, sqdist))
      {
        pc.points.push_back(pp);
      }
    }
    pub_debug_.publish(pc);

    if (output_pcd_)
      *pc_all_accum_ += *pc_particle;

    if (msg->header.stamp - match_output_last_ > *params_.match_output_interval &&
        (pub_matched_.getNumSubscribers() > 0 || pub_unmatched_.getNumSubscribers() > 0))
    {
      match_output_last_ = msg->header.stamp;

      sensor_msgs::PointCloud pc_match;
      pc_match.header.stamp = msg->header.stamp;
      pc_match.header.frame_id = frame_ids_["map"];
      sensor_msgs::PointCloud pc_unmatch;
      pc_unmatch.header.stamp = msg->header.stamp;
      pc_unmatch.header.frame_id = frame_ids_["map"];

      e.transform(*pc_local_full);

      const double match_dist_sq = params_.match_output_dist * params_.match_output_dist;
      for (auto &p : pc_local_full->points)
      {
        geometry_msgs::Point32 pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;

        if (!kdtree_->radiusSearch(p, params_.unmatch_output_dist, id, sqdist, 1))
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
    pa.header.stamp = odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance;
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

    pf_->resample(State(
        Vec3(params_.resample_var_x,
             params_.resample_var_y,
             params_.resample_var_z),
        Vec3(params_.resample_var_roll,
             params_.resample_var_pitch,
             params_.resample_var_yaw)));

    std::normal_distribution<float> noise(0.0, 1.0);
    auto update_noise_func = [this, &noise](State &s)
    {
      s.noise_ll_ = noise(engine_) * params_.odom_err_lin_lin;
      s.noise_la_ = noise(engine_) * params_.odom_err_lin_ang;
      s.noise_aa_ = noise(engine_) * params_.odom_err_ang_ang;
      s.noise_al_ = noise(engine_) * params_.odom_err_ang_lin;
    };
    pf_->predict(update_noise_func);

    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("MCL (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    const auto e_max = pf_->max();
    ROS_DEBUG("odom error integral: %0.3f, %0.3f, %0.3f",
              e_max.odom_err_integ.x,
              e_max.odom_err_integ.y,
              e_max.odom_err_integ.z);
    pc_local_accum_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pc_accum_header_.clear();

    ros::Time localized_current = ros::Time::now();
    float dt = (localized_current - localized_last_).toSec();
    if (dt > 1.0)
      dt = 1.0;
    tf_tolerance_base_ = ros::Duration(localize_rate_->in(dt));
    localized_last_ = localized_current;

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles)
    {
      const int reduced = pf_->getParticleSize() * 0.75;
      if (reduced > params_.num_particles)
      {
        pf_->resizeParticle(reduced);
      }
      else
      {
        pf_->resizeParticle(params_.num_particles);
      }
    }
  }
  void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
  {
    Vec3 acc;
    acc.x = f_acc_[0]->in(msg->linear_acceleration.x);
    acc.y = f_acc_[1]->in(msg->linear_acceleration.y);
    acc.z = f_acc_[2]->in(msg->linear_acceleration.z);

    float dt = (msg->header.stamp - imu_last_).toSec();
    if (dt < 0.0)
    {
      f_acc_[0]->set(0.0);
      f_acc_[1]->set(0.0);
      f_acc_[2]->set(0.0);
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
      NormalLikelihood<float> nd(params_.acc_var);
      auto imu_measure_func = [this, &nd, &acc_measure, &acc_measure_norm](const State &s) -> float
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
  bool cbResizeParticle(mcl_3dl::ResizeParticleRequest &request,
                        mcl_3dl::ResizeParticleResponse &response)
  {
    pf_->resizeParticle(request.size);
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
        params_.global_localization_grid,
        params_.global_localization_grid,
        params_.global_localization_grid);
    ds.filter(*points);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree->setInputCloud(points);

    auto pc_filter = [this, kdtree](const pcl::PointXYZI &p)
    {
      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      auto p2 = p;
      p2.z += 0.01 + params_.global_localization_grid;

      return kdtree->radiusSearch(
          p2, params_.global_localization_grid, id, sqdist, 1);
    };
    points->erase(
        std::remove_if(points->begin(), points->end(), pc_filter),
        points->end());

    const int dir = params_.global_localization_div_yaw;
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
    : engine_(seed_gen_())
  {
    ros::NodeHandle nh("~");

    sub_cloud_ = nh.subscribe("cloud", 100, &MCL3dlNode::cbCloud, this);
    sub_odom_ = nh.subscribe("odom", 200, &MCL3dlNode::cbOdom, this);
    sub_imu_ = nh.subscribe("imu", 200, &MCL3dlNode::cbImu, this);
    sub_mapcloud_ = nh.subscribe("mapcloud", 1, &MCL3dlNode::cbMapcloud, this);
    sub_mapcloud_update_ = nh.subscribe("mapcloud_update", 1, &MCL3dlNode::cbMapcloudUpdate, this);
    sub_position_ = nh.subscribe("initialpose", 1, &MCL3dlNode::cbPosition, this);
    pub_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 5, false);
    pub_particle_ = nh.advertise<geometry_msgs::PoseArray>("particles", 1, true);
    pub_debug_ = nh.advertise<sensor_msgs::PointCloud>("debug", 5, true);
    pub_mapcloud_ = nh.advertise<sensor_msgs::PointCloud2>("updated_map", 1, true);
    pub_debug_marker_ = nh.advertise<visualization_msgs::MarkerArray>("debug_marker", 1, true);
    srv_particle_size_ = nh.advertiseService("resize_particle", &MCL3dlNode::cbResizeParticle, this);
    srv_global_localization_ = nh.advertiseService("global_localization", &MCL3dlNode::cbGlobalLocalization, this);

    nh.param("map_frame", frame_ids_["map"], std::string("map"));
    nh.param("robot_frame", frame_ids_["base_link"], std::string("base_link"));
    nh.param("odom_frame", frame_ids_["odom"], std::string("odom"));
    nh.param("floor_frame", frame_ids_["floor"], std::string("floor"));
    nh.param("clip_near", params_.clip_near, 0.5);
    nh.param("clip_far", params_.clip_far, 10.0);
    params_.clip_near_sq = pow(params_.clip_near, 2.0);
    params_.clip_far_sq = pow(params_.clip_far, 2.0);
    nh.param("clip_z_min", params_.clip_z_min, -2.0);
    nh.param("clip_z_max", params_.clip_z_max, 2.0);
    nh.param("clip_beam_near", params_.clip_beam_near, 0.5);
    nh.param("clip_beam_far", params_.clip_beam_far, 4.0);
    params_.clip_beam_near_sq = pow(params_.clip_beam_near, 2.0);
    params_.clip_beam_far_sq = pow(params_.clip_beam_far, 2.0);
    nh.param("clip_beam_z_min", params_.clip_beam_z_min, -2.0);
    nh.param("clip_beam_z_max", params_.clip_beam_z_max, 2.0);
    nh.param("map_clip_z_min", params_.map_clip_z_min, -3.0);
    nh.param("map_clip_z_max", params_.map_clip_z_max, 3.0);
    nh.param("map_clip_far", params_.map_clip_far, 40.0);
    params_.map_clip_far_sq = pow(params_.map_clip_far, 2.0);
    nh.param("map_downsample_x", params_.map_downsample_x, 0.1);
    nh.param("map_downsample_y", params_.map_downsample_y, 0.1);
    nh.param("map_downsample_z", params_.map_downsample_z, 0.1);
    nh.param("downsample_x", params_.downsample_x, 0.1);
    nh.param("downsample_y", params_.downsample_y, 0.1);
    nh.param("downsample_z", params_.downsample_z, 0.05);
    params_.map_grid_min = std::min(std::min(params_.map_downsample_x, params_.map_downsample_y),
                                    params_.map_downsample_z);
    nh.param("update_downsample_x", params_.update_downsample_x, 0.3);
    nh.param("update_downsample_y", params_.update_downsample_y, 0.3);
    nh.param("update_downsample_z", params_.update_downsample_z, 0.3);
    double map_update_interval_t;
    nh.param("map_update_interval_interval", map_update_interval_t, 2.0);
    params_.map_update_interval.reset(new ros::Duration(map_update_interval_t));

    nh.param("match_dist_min", params_.match_dist_min, 0.2);
    nh.param("match_weight", params_.match_weight, 5.0);

    double weight[3];
    float weight_f[4];
    nh.param("dist_weight_x", weight[0], 1.0);
    nh.param("dist_weight_y", weight[1], 1.0);
    nh.param("dist_weight_z", weight[2], 5.0);
    for (size_t i = 0; i < 3; i++)
      weight_f[i] = weight[i];
    weight_f[3] = 0.0;
    point_rep_.setRescaleValues(weight_f);

    nh.param("global_localization_grid_lin", params_.global_localization_grid, 0.3);
    double grid_ang;
    nh.param("global_localization_grid_ang", grid_ang, 0.524);
    params_.global_localization_div_yaw = lroundf(2 * M_PI / grid_ang);

    nh.param("num_particles", params_.num_particles, 64);
    pf_.reset(new pf::ParticleFilter<State, float, ParticleWeightedMeanQuat>(params_.num_particles));
    nh.param("num_points", params_.num_points, 96);
    nh.param("num_points_global", params_.num_points_global, 8);
    nh.param("num_points_beam", params_.num_points_beam, 3);

    nh.param("beam_likelihood", params_.beam_likelihood_min, 0.2);
    params_.beam_likelihood = powf(params_.beam_likelihood_min, 1.0 / static_cast<float>(params_.num_points_beam));
    double ang_total_ref;
    nh.param("ang_total_ref", ang_total_ref, M_PI / 6.0);
    params_.sin_total_ref = sinf(ang_total_ref);

    nh.param("resample_var_x", params_.resample_var_x, 0.05);
    nh.param("resample_var_y", params_.resample_var_y, 0.05);
    nh.param("resample_var_z", params_.resample_var_z, 0.05);
    nh.param("resample_var_roll", params_.resample_var_roll, 0.05);
    nh.param("resample_var_pitch", params_.resample_var_pitch, 0.05);
    nh.param("resample_var_yaw", params_.resample_var_yaw, 0.05);

    nh.param("odom_err_lin_lin", params_.odom_err_lin_lin, 0.10);
    nh.param("odom_err_lin_ang", params_.odom_err_lin_ang, 0.05);
    nh.param("odom_err_ang_lin", params_.odom_err_ang_lin, 0.05);
    nh.param("odom_err_ang_ang", params_.odom_err_ang_ang, 0.05);

    nh.param("odom_err_integ_tc", params_.odom_err_integ_tc, 10.0);
    nh.param("odom_err_integ_sigma", params_.odom_err_integ_sigma, 100.0);

    double x, y, z;
    double roll, pitch, yaw;
    double v_x, v_y, v_z;
    double v_roll, v_pitch, v_yaw;
    nh.param("init_x", x, 0.0);
    nh.param("init_y", y, 0.0);
    nh.param("init_z", z, 0.0);
    nh.param("init_roll", roll, 0.0);
    nh.param("init_pitch", pitch, 0.0);
    nh.param("init_yaw", yaw, 0.0);
    nh.param("init_var_x", v_x, 2.0);
    nh.param("init_var_y", v_y, 2.0);
    nh.param("init_var_z", v_z, 0.5);
    nh.param("init_var_roll", v_roll, 0.1);
    nh.param("init_var_pitch", v_pitch, 0.1);
    nh.param("init_var_yaw", v_yaw, 0.5);
    pf_->init(
        State(
            Vec3(x, y, z),
            Quat(Vec3(roll, pitch, yaw))),
        State(
            Vec3(v_x, v_y, v_z),
            Vec3(v_roll, v_pitch, v_yaw)));

    double lpf_step;
    nh.param("lpf_step", lpf_step, 16.0);
    f_pos_[0].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    f_pos_[1].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    f_pos_[2].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    f_ang_[0].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0, true));
    f_ang_[1].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0, true));
    f_ang_[2].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0, true));

    nh.param("acc_lpf_step", lpf_step, 128.0);
    f_acc_[0].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    f_acc_[1].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    f_acc_[2].reset(new Filter(Filter::FILTER_LPF, lpf_step, 0.0));
    nh.param("acc_var", params_.acc_var, M_PI / 4.0);  // 45 deg

    nh.param("jump_dist", params_.jump_dist, 1.0);
    nh.param("jump_ang", params_.jump_ang, 1.57);
    nh.param("fix_dist", params_.fix_dist, 0.2);
    nh.param("fix_ang", params_.fix_ang, 0.1);
    nh.param("bias_var_dist", params_.bias_var_dist, 2.0);
    nh.param("bias_var_ang", params_.bias_var_ang, 1.57);

    nh.param("skip_measure", params_.skip_measure, 1);
    cnt_measure_ = 0;
    nh.param("accum_cloud", params_.accum_cloud, 1);
    cnt_accum_ = 0;

    nh.param("match_output_dist", params_.match_output_dist, 0.1);
    nh.param("unmatch_output_dist", params_.unmatch_output_dist, 0.5);
    double match_output_interval_t;
    nh.param("match_output_interval_interval", match_output_interval_t, 0.2);
    params_.match_output_interval.reset(new ros::Duration(match_output_interval_t));
    pub_matched_ = nh.advertise<sensor_msgs::PointCloud2>("matched", 2, true);
    pub_unmatched_ = nh.advertise<sensor_msgs::PointCloud2>("unmatched", 2, true);

    double tf_tolerance_t;
    nh.param("tf_tolerance", tf_tolerance_t, 0.05);
    params_.tf_tolerance.reset(new ros::Duration(tf_tolerance_t));

    nh.param("publish_tf", publish_tf_, true);
    nh.param("output_pcd", output_pcd_, false);

    imu_quat_ = Quat(0.0, 0.0, 0.0, 1.0);

    has_odom_ = has_map_ = false;
    match_output_last_ = ros::Time::now();
    localize_rate_.reset(new Filter(Filter::FILTER_LPF, 5.0, 0.0));
    localized_last_ = ros::Time::now();

    map_update_timer_ = nh.createTimer(
        *params_.map_update_interval,
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
      const auto e = state_prev_;
      *pc_map2_ = *pc_map_ + *pc_update_;

      auto pc_map_filter = [this, &e](const pcl::PointXYZI &p)
      {
        if (p.z - e.pos.z > params_.map_clip_z_max)
          return true;
        if (p.z - e.pos.z < params_.map_clip_z_min)
          return true;
        if (powf(p.x - e.pos.x, 2.0) + powf(p.y - e.pos.y, 2.0) > params_.map_clip_far_sq)
          return true;
        return false;
      };
      pc_map2_->erase(
          std::remove_if(pc_map2_->begin(), pc_map2_->end(), pc_map_filter),
          pc_map2_->end());

      kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
      kdtree_->setEpsilon(params_.map_grid_min);
      kdtree_->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep_));

      if (pc_map2_->points.size() == 0)
        kdtree_->setInputCloud(pc_map_);
      else
        kdtree_->setInputCloud(pc_map2_);

      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*pc_map2_, out);
      pub_mapcloud_.publish(out);
    }
  }

protected:
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_mapcloud_;
  ros::Subscriber sub_mapcloud_update_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_position_;
  ros::Publisher pub_particle_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_mapcloud_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_matched_;
  ros::Publisher pub_unmatched_;
  ros::Publisher pub_debug_marker_;
  ros::Timer map_update_timer_;
  ros::ServiceServer srv_particle_size_;
  ros::ServiceServer srv_global_localization_;

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
  State odom_;
  State odom_prev_;
  std::map<std::string, bool> frames_;
  std::vector<std::string> frames_v_;
  size_t frame_num_;
  State state_prev_;
  ros::Time imu_last_;
  int cnt_measure_;
  int cnt_accum_;
  Quat imu_quat_;

  MyPointRepresentation point_rep_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map2_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_update_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_all_accum_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_accum_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_orig_;
  std::vector<std_msgs::Header> pc_accum_header_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mcl_3dl");

  MCL3dlNode mcl(argc, argv);
  ros::spin();

  return 0;
}

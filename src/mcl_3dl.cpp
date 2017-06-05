#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>

#include <pf.hpp>
#include <vec3.hpp>
#include <quat.hpp>
#include <filter.hpp>
#include <nd.hpp>


class mcl_3dl_node
{
private:
	ros::Subscriber sub_cloud;
	ros::Subscriber sub_mapcloud;
	ros::Subscriber sub_mapcloud_update;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_imu;
	ros::Subscriber sub_position;
	ros::Publisher pub_particle;
	ros::Publisher pub_debug;
	ros::Publisher pub_mapcloud;
	ros::Publisher pub_pose;
	ros::Publisher pub_matched;
	ros::Publisher pub_unmatched;
	ros::Publisher pub_debug_marker;

	tf::TransformListener tfl;
	tf::TransformBroadcaster tfb;

	std::shared_ptr<filter> f_pos[3]; 
	std::shared_ptr<filter> f_ang[3]; 
	std::shared_ptr<filter> f_acc[3]; 
	std::shared_ptr<filter> localize_rate; 
	ros::Time localized_last;
	ros::Duration tf_tolerance_base;

	class parameters
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
		double map_downsample_x;
		double map_downsample_y;
		double map_downsample_z;
		double update_downsample_x;
		double update_downsample_y;
		double update_downsample_z;
		double map_grid_min;
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
		std::shared_ptr<ros::Duration> match_output_interval;
		std::shared_ptr<ros::Duration> tf_tolerance;
	} params;
	int cnt_measure;
	int cnt_accum;
	ros::Time match_output_last;
	bool output_pcd;
	bool publish_tf;

	class state: public pf::particleBase<float>
	{
	public:
		vec3 pos;
		quat rot;
		bool diff;
		struct
		{
			vec3 lin;
			vec3 ang;
		} vel;
		class rpy_vec
		{
		public:
			vec3 v;
			rpy_vec()
			{
			}
			rpy_vec(const vec3 &v)
			{
				this->v = v;
			}
			rpy_vec(const float &r, const float &p, const float y)
			{
				this->v.x = r;
				this->v.y = p;
				this->v.z = y;
			}
		} rpy; 
		virtual float &operator[](const size_t i) override
		{
			switch(i)
			{
			case 0: return pos.x;
			case 1: return pos.y;
			case 2: return pos.z;
			case 3: return rot.x;
			case 4: return rot.y;
			case 5: return rot.z;
			case 6: return rot.w;
			case 7: return vel.lin.x;
			case 8: return vel.lin.y;
			case 9: return vel.lin.z;
			case 10: return vel.ang.x;
			case 11: return vel.ang.y;
			case 12: return vel.ang.z;
			}
			return pos.x;
		};
		virtual const size_t size() override
		{
			return 13;
		};
		virtual void normalize() override
		{
			rot.normalize();
		};
		state()
		{
			diff = false;
		};
		state(const vec3 pos, const quat rot)
		{
			this->pos = pos;
			this->rot = rot;
			diff = false;
		};
		state(const vec3 pos, const vec3 rpy)
		{
			this->pos = pos;
			this->rpy = rpy_vec(rpy);
			diff = true;
		};
		state(const vec3 pos, const quat rot, const vec3 lin, const vec3 ang)
		{
			this->pos = pos;
			this->rot = rot;
			this->vel.lin = lin;
			this->vel.ang = ang;
			diff = false;
		};
		bool is_diff()
		{
			return diff;
		}
		void transform(pcl::PointCloud<pcl::PointXYZI> &pc) const
		{
			auto r = rot.normalized();
			for(auto &p: pc.points)
			{
				auto t = r * vec3(p.x, p.y, p.z) + pos;
				p.x = t.x;
				p.y = t.y;
				p.z = t.z;
			}
		}
		state generate_noise(
				std::default_random_engine &engine,
				state mean, state sigma)
		{
			state noise;
			if(mean.is_diff() || !sigma.is_diff())
			{
				ROS_ERROR("Failed to generate noise. mean must be quat and sigma must be rpy vec.");
			}
			for(size_t i = 0; i < size(); i ++)
			{
				if(3 <= i && i <= 6) continue;
				std::normal_distribution<float> nd(mean[i], sigma[i]);
				noise[i] = nd(engine);
			}
			vec3 rpy_noise;
			for(size_t i = 0; i < 3; i ++)
			{
				std::normal_distribution<float> nd(0.0, sigma.rpy.v[i]);
				rpy_noise[i] = nd(engine);
			}
			noise.rot = quat(rpy_noise) * mean.rot;
			return noise;
		}
		state operator+(const state &a)
		{
			state in = a;
			state ret;
			for(size_t i = 0; i < size(); i ++)
			{
				if(3 <= i && i <= 6) continue;
				ret[i] = (*this)[i] + in[i];
			}
			ret.rot = a.rot * rot;
			return ret;
		}
		void weight(const float &s)
		{
			for(size_t i = 0; i < size(); i ++)
			{
				if(3 <= i && i <= 6) continue;
				(*this)[i] = (*this)[i] * s;
			}
			vec3 axis;
			float ang;
			rot.get_axis_ang(axis, ang);
			rot.set_axis_ang(axis, ang * s);
		}
	};
	std::shared_ptr<pf::particle_filter<state>> pf;

	class MyPointRepresentation: public pcl::PointRepresentation<pcl::PointXYZI>
	{
		using pcl::PointRepresentation<pcl::PointXYZI>::nr_dimensions_;
	public:
		MyPointRepresentation ()
		{
			nr_dimensions_ = 3;
		}

		virtual void copyToFloatArray(const pcl::PointXYZI &p, float * out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
		}
	};
	MyPointRepresentation point_rep;

	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_update;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_all_accum;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_orig;
	void cb_mapcloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		ROS_INFO("map received");
		pcl::PointCloud<pcl::PointXYZI> pc_tmp;
		pcl::fromROSMsg(*msg, pc_tmp);

		pc_map.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pc_map2.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pc_update.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::VoxelGrid<pcl::PointXYZI> ds;
		ds.setInputCloud(pc_tmp.makeShared());
		ds.setLeafSize(params.map_downsample_x, params.map_downsample_y, params.map_downsample_z);
		ds.filter(*pc_map);
		pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pc_all_accum.reset(new pcl::PointCloud<pcl::PointXYZI>);
		frame_num = 0;
		has_map = true;

		kdtree_orig.reset(new  pcl::KdTreeFLANN<pcl::PointXYZI>);
		kdtree_orig->setInputCloud(pc_map);
		std::vector<int> id(7);
		std::vector<float> sqdist(7);
		ROS_INFO("map original: %d points", (int)pc_map->points.size());

		// map pointcloud filters here

		pc_map->width = 1;
		pc_map->height = pc_map->points.size();
		ROS_INFO("map reduced: %d points", (int)pc_map->points.size());
		kdtree_orig.reset(new  pcl::KdTreeFLANN<pcl::PointXYZI>);
		kdtree_orig->setInputCloud(pc_map);

		*pc_map2 = *pc_map;
		kdtree.reset(new  pcl::KdTreeFLANN<pcl::PointXYZI>);
		kdtree->setEpsilon(params.map_grid_min / 2);
		kdtree->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep));
		kdtree->setInputCloud(pc_map2);
	}
	void cb_mapcloud_update(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		ROS_INFO("map_update received");
		pcl::PointCloud<pcl::PointXYZI> pc_tmp;
		pcl::fromROSMsg(*msg, pc_tmp);

		pc_update.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::VoxelGrid<pcl::PointXYZI> ds;
		ds.setInputCloud(pc_tmp.makeShared());
		ds.setLeafSize(params.update_downsample_x, params.update_downsample_y, params.update_downsample_z);
		ds.filter(*pc_update);
	}

	void cb_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		geometry_msgs::PoseStamped pose_in, pose;
		pose_in.header = msg->header;
		pose_in.pose = msg->pose.pose;
		try
		{
			tfl.waitForTransform(frame_ids["map"], pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(1.0));
			tfl.transformPose(frame_ids["map"], pose_in, pose);
		}
		catch(tf::TransformException &e)
		{
			return;
		}
		pf->init(
				state(
					vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
					quat(pose.pose.orientation.x,
						pose.pose.orientation.y,
						pose.pose.orientation.z,
						pose.pose.orientation.w)
					), 
				state(
					vec3(msg->pose.covariance[0], 
						msg->pose.covariance[6*1+1],
						msg->pose.covariance[6*2+2]),
					vec3(msg->pose.covariance[6*3+3],
						msg->pose.covariance[6*4+4],
						msg->pose.covariance[6*5+5])
					));
		pc_update.reset(new pcl::PointCloud<pcl::PointXYZI>);
	}

	std::map<std::string, std::string> frame_ids;

	ros::Time odom_last;
	bool has_map;
	bool has_odom;
	state odom;
	state odom_prev;
	void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
	{
		odom =
			state(
					vec3(msg->pose.pose.position.x,
						msg->pose.pose.position.y,
						msg->pose.pose.position.z),
					quat(msg->pose.pose.orientation.x,
						msg->pose.pose.orientation.y,
						msg->pose.pose.orientation.z,
						msg->pose.pose.orientation.w)
				 );
		if(has_odom)
		{
			float dt = (msg->header.stamp - odom_last).toSec();
			if(dt < 0.0)
			{
				has_odom = false;
			}
			else if(dt > 0.05)
			{
			//	ROS_INFO("dt %0.3f", dt);
				vec3 v = odom_prev.rot.inv() * (odom.pos - odom_prev.pos);
				quat r = odom.rot * odom_prev.rot.inv();
				r.rotate_axis(odom_prev.rot.inv());
				vec3 axis;
				float ang;
				r.get_axis_ang(axis, ang);
				r.normalize();

				float trans = v.norm();
				std::normal_distribution<float> ll(1.0, trans * params.odom_err_lin_lin);
				std::normal_distribution<float> la(1.0, trans * params.odom_err_lin_ang);
				std::normal_distribution<float> al(1.0, fabs(ang) * params.odom_err_ang_lin);
				std::normal_distribution<float> aa(1.0, fabs(ang) * params.odom_err_ang_ang);
				pf->predict([&](state &s)
						{
							s.rot.normalize();
							quat r2 = r;
							r2.rotate_axis(s.rot);
							s.rot = (r2 * aa(engine) * la(engine)) * s.rot;
							s.pos += s.rot * (v * ll(engine) * al(engine));
						});
				odom_last = msg->header.stamp;
				odom_prev = odom;
			}
		}
		else
		{
			odom_prev = odom;
			odom_last = msg->header.stamp;
			has_odom = true;
		}
	}
	std::random_device seed_gen;
	std::default_random_engine engine;
	std::map<std::string, bool> frames;
	std::vector<std::string> frames_v;
	size_t frame_num;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_accum;
	std::vector<std_msgs::Header> pc_accum_header;
	state state_prev;
	void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		if(!has_map) return;
		if(frames.find(msg->header.frame_id) == frames.end())
		{
			frames[msg->header.frame_id] = true;
			frames_v.push_back(msg->header.frame_id);
		}

		sensor_msgs::PointCloud2 pc_bl;
		if(!pcl_ros::transformPointCloud(frame_ids["odom"], *msg, pc_bl, tfl))
		{
			return;
		}
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_tmp(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(pc_bl, *pc_tmp);
		for(auto &p: pc_tmp->points)
		{
			p.intensity = pc_accum_header.size();
		}
		*pc_local_accum += *pc_tmp;
		pc_local_accum->header.frame_id = frame_ids["odom"];
		pc_accum_header.push_back(msg->header);

		if(frames_v[frame_num].compare(msg->header.frame_id) != 0) return;
		frame_num ++;
		if(frame_num >= frames_v.size()) frame_num = 0;

		//ROS_INFO("cloud %s  %d/%d", msg->header.frame_id.c_str(), frame_num, frames_v.size());

		if(frame_num != 0) return;

		cnt_accum ++;
		if(cnt_accum % params.accum_cloud != 0) return;

		cnt_measure ++;
		if(cnt_measure % params.skip_measure != 0)
		{
			pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZI>);
			pc_accum_header.clear();
			return;
		}

		if(!pcl_ros::transformPointCloud(
					frame_ids["base_link"], *pc_local_accum, *pc_local_accum, tfl))
		{
			ROS_ERROR("Failed to transform laser origin.");
			pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZI>);
			pc_accum_header.clear();
			return;
		}
		std::vector<vec3> origins;
		for(auto &h: pc_accum_header)
		{
			try
			{
				tf::StampedTransform trans;
				tfl.waitForTransform(frame_ids["base_link"], msg->header.stamp, 
						h.frame_id, h.stamp, 
						frame_ids["odom"], ros::Duration(0.05));
				tfl.lookupTransform(frame_ids["base_link"], msg->header.stamp,
						h.frame_id, h.stamp, 
						frame_ids["odom"], trans);
				auto origin = trans.getOrigin();
				origins.push_back(vec3(origin.x(), origin.y(), origin.z()));
			}
			catch(tf::TransformException &e)
			{
				ROS_ERROR("Failed to transform laser origin.");
				origins.push_back(vec3(0.0, 0.0, 0.0));
			}
			/*ROS_INFO(" beam_origin[%d]  %0.3f %0.3f %0.3f",
					(int)origins.size() - 1, 
					origins.back().x,
					origins.back().y,
					origins.back().z);*/
		}

		const auto ts = std::chrono::high_resolution_clock::now();

		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::VoxelGrid<pcl::PointXYZI> ds;
		ds.setInputCloud(pc_local_accum);
		ds.setLeafSize(params.downsample_x, params.downsample_y, params.downsample_z);
		ds.filter(*pc_local);
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_full(new pcl::PointCloud<pcl::PointXYZI>);
		*pc_local_full = *pc_local;

		std::uniform_real_distribution<float> ud(0.0, 1.0);
		int num_valid = 0;
		for(auto &p: pc_local->points)
		{
			if(p.x*p.x + p.y*p.y > params.clip_far_sq) continue;
			if(p.x*p.x + p.y*p.y < params.clip_near_sq) continue;
			if(p.z < params.clip_z_min || params.clip_z_max < p.z) continue;
			num_valid ++;
		}
		float use_rate = (float)params.num_points / num_valid;
		pc_local->points.erase(
				std::remove_if(pc_local->points.begin(), pc_local->points.end(),
					[&](const pcl::PointXYZI &p)
					{
						if(p.x*p.x + p.y*p.y > params.clip_far_sq) return true;
						if(p.x*p.x + p.y*p.y < params.clip_near_sq) return true;
						if(p.z < params.clip_z_min || params.clip_z_max < p.z) return true;
						return ud(engine) > use_rate;
					}), pc_local->points.end());
		pc_local->width = 1;
		pc_local->height = pc_local->points.size();
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_local_beam(new pcl::PointCloud<pcl::PointXYZI>);
		*pc_local_beam = *pc_local;
		pc_local_beam->points.erase(
				std::remove_if(pc_local_beam->points.begin(), pc_local_beam->points.end(),
					[&](const pcl::PointXYZI &p)
					{
						if(p.x*p.x + p.y*p.y > params.clip_beam_far_sq) return true;
						if(p.x*p.x + p.y*p.y < params.clip_beam_near_sq) return true;
						if(p.z < params.clip_beam_z_min || params.clip_beam_z_max < p.z) return true;
						return false;
					}), pc_local_beam->points.end());
		float use_rate_beam = (float)params.num_points_beam / pc_local->points.size();
		pc_local_beam->points.erase(
				std::remove_if(pc_local_beam->points.begin(), pc_local_beam->points.end(),
					[&](const pcl::PointXYZI &p)
					{
						if(p.intensity - roundf(p.intensity) > 0.01)
							return true;
						return ud(engine) > use_rate_beam;
					}), pc_local_beam->points.end());
		pc_local_beam->width = 1;
		pc_local_beam->height = pc_local_beam->points.size();

		/*for(auto p: pc_local_beam->points)
		{
			int beam_header_id = lroundf(p.intensity);
			ROS_INFO(" raycast point origin %d (%0.3f %0.3f %0.3f)", 
					beam_header_id, p.x, p.y, p.z);
		}*/

		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_particle_beam(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_dummy(new pcl::PointCloud<pcl::PointXYZI>);

		std::vector<int> id(1);
		std::vector<float> sqdist(1);

		const float match_dist_min = params.match_dist_min;
		const float match_weight = params.match_weight;
		pf->measure([&](const state &s)->float
				{
					// likelihood model
					float score_like = 0;
					*pc_particle = *pc_local;
					s.transform(*pc_particle);
					size_t num = 0;
					for(auto &p: pc_particle->points)
					{
						if(kdtree->radiusSearch(p, match_dist_min, id, sqdist, 1))
						{
							float dist = sqdist[0];
							if(dist < 0.05 * 0.05) dist = 0.05 * 0.05;
							dist = match_dist_min - sqrtf(dist);
							if(dist < 0.0) continue;

							score_like += dist * match_weight;
							num ++;
						}
					}

					// approximative beam model
					float score_beam = 1.0;
					*pc_particle_beam = *pc_local_beam;
					s.transform(*pc_particle_beam);
					for(auto &p: pc_particle_beam->points)
					{
						int beam_header_id = lroundf(p.intensity);
						vec3 pos = s.pos + s.rot * origins[beam_header_id];
						vec3 end(p.x, p.y, p.z);
						int num = (end - pos).norm() / params.map_grid_min;
						vec3 inc = (end - pos) / num;
						for(int i = 0; i < num - 1; i ++)
						{
							pos += inc;
							pcl::PointXYZI center;
							center.x = pos.x;
							center.y = pos.y;
							center.z = pos.z;
							center.intensity = 0.0;
							if(kdtree->radiusSearch(center, 
										params.map_grid_min / 2.0, id, sqdist, 1))
							{
								float d0 = sqrtf(sqdist[0]);
								vec3 pos_prev = pos - (inc * 2.0);
								pcl::PointXYZI center_prev;
								center_prev.x = pos_prev.x;
								center_prev.y = pos_prev.y;
								center_prev.z = pos_prev.z;
								center_prev.intensity = 0.0;
								kdtree->nearestKSearch(center_prev, 1, id, sqdist);
								float d1 = sqrtf(sqdist[0]);

								float sin_ang = (d1 - d0) / (inc.norm() * 2.0);
								// reject total reflection
								if(sin_ang > params.sin_total_ref || fabs(d1 - d0) < 1e-6)
								{
									score_beam *= params.beam_likelihood;
								}
								break;
							}
						}
					}
					if(score_beam < params.beam_likelihood_min)
						score_beam = params.beam_likelihood_min;
					
					return score_like * score_beam;
				});

		normal_likelihood<float> nl_lin((float)params.bias_var_dist);
		normal_likelihood<float> nl_ang((float)params.bias_var_ang);
		pf->bias([&](const state &s, float &p_bias)->void
				{
					const float lin_diff = (s.pos - state_prev.pos).norm();
					vec3 axis;
					float ang_diff;
					(s.rot * state_prev.rot.inv()).get_axis_ang(axis, ang_diff);
					p_bias = nl_lin(lin_diff) * nl_ang(ang_diff);
				});
		auto e = pf->max_biased();
		e.rot.normalize();

		{
			visualization_msgs::MarkerArray markers;

			*pc_particle_beam = *pc_local_beam;
			e.transform(*pc_particle_beam);
			for(auto &p: pc_particle_beam->points)
			{
				int beam_header_id = lroundf(p.intensity);
				vec3 pos = e.pos + e.rot * origins[beam_header_id];
				vec3 end(p.x, p.y, p.z);

				visualization_msgs::Marker marker;
				marker.header.frame_id = frame_ids["map"];
				marker.header.stamp = msg->header.stamp;
				marker.ns = "Rays";
				marker.id = markers.markers.size();
				marker.type= visualization_msgs::Marker::LINE_STRIP;
				marker.action = 0;
				marker.pose.position.x = 0.0;
				marker.pose.position.y = 0.0;
				marker.pose.position.z = 0.0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = marker.scale.y = marker.scale.z = 0.025;
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
			pub_debug_marker.publish(markers);
		}

		vec3 map_pos;
		quat map_rot;
		map_pos = e.pos - e.rot * odom.rot.inv() * odom.pos;
		map_rot = e.rot * odom.rot.inv();

		bool jump = false;
		{
			vec3 jump_axis;
			float jump_ang;
			float jump_dist = (e.pos - state_prev.pos).norm();
			(e.rot.inv() * state_prev.rot).get_axis_ang(jump_axis, jump_ang);
			if(jump_dist > params.jump_dist ||
					fabs(jump_ang) > params.jump_ang)
			{
				ROS_INFO("Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
				jump = true;
			}
			state_prev = e;
		}
		tf::StampedTransform trans;
		trans.stamp_ = ros::Time::now() + tf_tolerance_base + *params.tf_tolerance;
		trans.frame_id_ = frame_ids["map"];
		trans.child_frame_id_ = frame_ids["odom"];
		auto rpy = map_rot.get_rpy();
		if(jump)
		{
			f_ang[0]->set(rpy.x);
			f_ang[1]->set(rpy.y);
			f_ang[2]->set(rpy.z);
			f_pos[0]->set(map_pos.x);
			f_pos[1]->set(map_pos.y);
			f_pos[2]->set(map_pos.z);
		}
		rpy.x = f_ang[0]->in(rpy.x);
		rpy.y = f_ang[1]->in(rpy.y);
		rpy.z = f_ang[2]->in(rpy.z);
		map_rot.set_rpy(rpy);
		map_pos.x = f_pos[0]->in(map_pos.x);
		map_pos.y = f_pos[1]->in(map_pos.y);
		map_pos.z = f_pos[2]->in(map_pos.z);
		trans.setOrigin(tf::Vector3(map_pos.x, map_pos.y, map_pos.z));
		trans.setRotation(tf::Quaternion(map_rot.x, map_rot.y, map_rot.z, map_rot.w));

		std::vector<tf::StampedTransform> transforms;
		transforms.push_back(trans);

		e.rot = map_rot * odom.rot;
		e.pos = map_pos + e.rot * odom.rot.inv() * odom.pos;

		trans.frame_id_ = frame_ids["map"];
		trans.child_frame_id_ = frame_ids["floor"];
		trans.setOrigin(tf::Vector3(0.0, 0.0, e.pos.z));
		trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

		transforms.push_back(trans);

		if(publish_tf) tfb.sendTransform(transforms);

		auto cov = pf->covariance();

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
		for(size_t i = 0; i < 36; i ++)
		{
			pose.pose.covariance[i] = cov[i/6][i%6];
		}
		pub_pose.publish(pose);

		bool fix = false;
		{
			vec3 fix_axis;
			float fix_ang = sqrtf(cov[3][3] + cov[4][4] + cov[5][5]);
			float fix_dist = sqrtf(cov[0][0] + cov[1][1] + cov[2][2]);
			ROS_DEBUG("cov: lin %0.3f ang %0.3f", fix_dist, fix_ang);
			if(fix_dist < params.fix_dist &&
					fabs(fix_ang) < params.fix_ang)
			{
				fix = true;
				ROS_DEBUG("Localization fixed");
			}
		}
		*pc_particle = *pc_local;
		e.transform(*pc_particle);
		sensor_msgs::PointCloud pc;
		pc.header.stamp = msg->header.stamp;
		pc.header.frame_id = frame_ids["map"];
		for(auto &p: pc_particle->points)
		{
			geometry_msgs::Point32 pp;
			pp.x = p.x;
			pp.y = p.y;
			pp.z = p.z;

			if(kdtree_orig->nearestKSearch(p, 1, id, sqdist))
			{
				pc.points.push_back(pp);
			}
		}
		pub_debug.publish(pc);
		
		if(output_pcd) *pc_all_accum += *pc_particle;

		if(msg->header.stamp - match_output_last > *params.match_output_interval &&
				(pub_matched.getNumSubscribers() > 0 || pub_unmatched.getNumSubscribers() > 0))
		{
			match_output_last = msg->header.stamp;

			sensor_msgs::PointCloud pc_match;
			pc_match.header.stamp = msg->header.stamp;
			pc_match.header.frame_id = frame_ids["map"];
			sensor_msgs::PointCloud pc_unmatch;
			pc_unmatch.header.stamp = msg->header.stamp;
			pc_unmatch.header.frame_id = frame_ids["map"];

			e.transform(*pc_local_full);

			const double match_dist_sq = params.match_output_dist * params.match_output_dist;
			for(auto &p: pc_local_full->points)
			{
				geometry_msgs::Point32 pp;
				pp.x = p.x;
				pp.y = p.y;
				pp.z = p.z;

				if(!kdtree->radiusSearch(p, params.unmatch_output_dist, id, sqdist, 1))
				{
					pc_unmatch.points.push_back(pp);
				}
				else if(sqdist[0] < match_dist_sq)
				{
					pc_match.points.push_back(pp);
				}
			}
			if(pub_matched.getNumSubscribers() > 0)
			{
				sensor_msgs::PointCloud2 pc2;
				sensor_msgs::convertPointCloudToPointCloud2(pc_match, pc2);
				pub_matched.publish(pc2);
			}
			if(pub_unmatched.getNumSubscribers() > 0)
			{
				sensor_msgs::PointCloud2 pc2;
				sensor_msgs::convertPointCloudToPointCloud2(pc_unmatch, pc2);
				pub_unmatched.publish(pc2);
			}
		}
			
		geometry_msgs::PoseArray pa;
		pa.header.stamp = ros::Time::now() + tf_tolerance_base + *params.tf_tolerance;
		pa.header.frame_id = frame_ids["map"];
		for(size_t i = 0; i < pf->get_particle_size(); i ++)
		{
			geometry_msgs::Pose pm;
			auto p = pf->get_particle(i);
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
		pub_particle.publish(pa);


		pf->resample(state(
					vec3(params.resample_var_x,
						params.resample_var_y,
						params.resample_var_z),
					vec3(params.resample_var_roll,
						params.resample_var_pitch,
						params.resample_var_yaw)
					)
				);

		const auto tnow = std::chrono::high_resolution_clock::now();
		ROS_INFO("MCL (%0.3f sec.)",
				std::chrono::duration<float>(tnow - ts).count());
		pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZI>);
		pc_accum_header.clear();


		ros::Time localized_current = ros::Time::now();
		float dt = (localized_current - localized_last).toSec();
		if(dt > 1.0) dt = 1.0;
		tf_tolerance_base = ros::Duration(localize_rate->in(dt));
		localized_last = localized_current;
	}
	ros::Time imu_last;
	void cb_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		vec3 acc;
		acc.x = f_acc[0]->in(msg->linear_acceleration.x);
		acc.y = f_acc[1]->in(msg->linear_acceleration.y);
		acc.z = f_acc[2]->in(msg->linear_acceleration.z);

		float dt = (msg->header.stamp - imu_last).toSec();
		if(dt < 0.0)
		{
			f_acc[0]->set(0.0);
			f_acc[1]->set(0.0);
			f_acc[2]->set(0.0);
		}
		else if(dt > 0.05)
		{
			vec3 acc_measure = acc / acc.norm();
			try
			{
				tfl.waitForTransform(msg->header.frame_id, frame_ids["base_link"], 
						msg->header.stamp, ros::Duration(0.1));
				tf::Stamped<tf::Vector3> in, out;
				in.frame_id_ = msg->header.frame_id;
				in.stamp_ = msg->header.stamp;
				in.setX(acc_measure.x);
				in.setY(acc_measure.y);
				in.setZ(acc_measure.z);
				tfl.transformVector(frame_ids["base_link"], in, out);
				acc_measure = vec3(out.x(), out.y(), out.z());
			}
			catch(tf::TransformException &e)
			{
				return;
			}
			float acc_measure_norm = acc_measure.norm();
			normal_likelihood<float> nd(params.acc_var);
			pf->measure([&](const state &s)->float
					{
						vec3 acc_estim = s.rot.inv() * vec3(0.0, 0.0, 1.0);
						float diff = acosf(
									acc_estim.dot(acc_measure)
									/ (acc_measure_norm * acc_estim.norm())
								);
						return nd(diff);
					});

			imu_last = msg->header.stamp;
		}
	}

public:
	mcl_3dl_node(int argc, char *argv[]):
		engine(seed_gen())
	{
		ros::NodeHandle nh("~");

		sub_cloud = nh.subscribe("cloud", 20, &mcl_3dl_node::cb_cloud, this);
		sub_odom = nh.subscribe("odom", 200, &mcl_3dl_node::cb_odom, this);
		sub_imu = nh.subscribe("imu", 200, &mcl_3dl_node::cb_imu, this);
		sub_mapcloud = nh.subscribe("mapcloud", 1, &mcl_3dl_node::cb_mapcloud, this);
		sub_mapcloud_update = nh.subscribe("mapcloud_update", 1, &mcl_3dl_node::cb_mapcloud_update, this);
		sub_position = nh.subscribe("initialpose", 1, &mcl_3dl_node::cb_position, this);
		pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 5, false);
		pub_particle = nh.advertise<geometry_msgs::PoseArray>("particles", 1, true);
		pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 5, true);
		pub_mapcloud = nh.advertise<sensor_msgs::PointCloud2>("updated_map", 1, true);
		pub_debug_marker = nh.advertise<visualization_msgs::MarkerArray>("debug_marker", 1, true);

		nh.param("map_frame", frame_ids["map"], std::string("map"));
		nh.param("robot_frame", frame_ids["base_link"], std::string("base_link"));
		nh.param("odom_frame", frame_ids["odom"], std::string("odom"));
		nh.param("floor_frame", frame_ids["floor"], std::string("floor"));
		nh.param("clip_near", params.clip_near, 0.5);
		nh.param("clip_far", params.clip_far, 10.0);
		params.clip_near_sq = pow(params.clip_near, 2.0);
		params.clip_far_sq = pow(params.clip_far, 2.0);
		nh.param("clip_z_min", params.clip_z_min, -2.0);
		nh.param("clip_z_max", params.clip_z_max, 2.0);
		nh.param("clip_beam_near", params.clip_beam_near, 0.5);
		nh.param("clip_beam_far", params.clip_beam_far, 4.0);
		params.clip_beam_near_sq = pow(params.clip_beam_near, 2.0);
		params.clip_beam_far_sq = pow(params.clip_beam_far, 2.0);
		nh.param("clip_beam_z_min", params.clip_beam_z_min, -2.0);
		nh.param("clip_beam_z_max", params.clip_beam_z_max, 2.0);
		nh.param("map_clip_z_min", params.map_clip_z_min, -3.0);
		nh.param("map_clip_z_max", params.map_clip_z_max, 3.0);
		nh.param("map_downsample_x", params.map_downsample_x, 0.1);
		nh.param("map_downsample_y", params.map_downsample_y, 0.1);
		nh.param("map_downsample_z", params.map_downsample_z, 0.1);
		nh.param("downsample_x", params.downsample_x, 0.1);
		nh.param("downsample_y", params.downsample_y, 0.1);
		nh.param("downsample_z", params.downsample_z, 0.05);
		params.map_grid_min = std::min(std::min(params.map_downsample_x, params.map_downsample_y), 
				params.map_downsample_z);
		nh.param("update_downsample_x", params.update_downsample_x, 0.3);
		nh.param("update_downsample_y", params.update_downsample_y, 0.3);
		nh.param("update_downsample_z", params.update_downsample_z, 0.3);
		double map_update_interval_t;
		nh.param("map_update_interval_interval", map_update_interval_t, 2.0);
		params.map_update_interval.reset(new ros::Duration(map_update_interval_t));
		
		nh.param("match_dist_min", params.match_dist_min, 0.2);
		nh.param("match_weight", params.match_weight, 5.0);

		double weight[3];
		float weight_f[4];
		nh.param("dist_weight_x", weight[0], 1.0);
		nh.param("dist_weight_y", weight[1], 1.0);
		nh.param("dist_weight_z", weight[2], 5.0);
		for(size_t i = 0; i < 3; i ++) weight_f[i] = weight[i];
		weight_f[3] = 0.0;
		point_rep.setRescaleValues(weight_f);

		nh.param("num_particles", params.num_particles, 64);
		pf.reset(new pf::particle_filter<state>(params.num_particles));
		nh.param("num_points", params.num_points, 96);
		nh.param("num_points_beam", params.num_points_beam, 3);
		
		nh.param("beam_likelihood", params.beam_likelihood_min, 0.2);
		params.beam_likelihood = powf(params.beam_likelihood_min, 1.0 / (float)params.num_points_beam);
		double ang_total_ref;
		nh.param("ang_total_ref", ang_total_ref, M_PI / 6.0);
		params.sin_total_ref = sinf(ang_total_ref);


		nh.param("resample_var_x", params.resample_var_x, 0.05);
		nh.param("resample_var_y", params.resample_var_y, 0.05);
		nh.param("resample_var_z", params.resample_var_z, 0.05);
		nh.param("resample_var_roll", params.resample_var_roll, 0.05);
		nh.param("resample_var_pitch", params.resample_var_pitch, 0.05);
		nh.param("resample_var_yaw", params.resample_var_yaw, 0.05);

		nh.param("odom_err_lin_lin", params.odom_err_lin_lin, 0.1);
		nh.param("odom_err_lin_ang", params.odom_err_lin_ang, 0.25);
		nh.param("odom_err_ang_lin", params.odom_err_ang_lin, 0.1);
		nh.param("odom_err_ang_ang", params.odom_err_ang_ang, 0.2);

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
		pf->init(
				state(
					vec3(x, y, z),
					quat(vec3(roll, pitch, yaw))
					), 
				state(
					vec3(v_x, v_y, v_z),
					vec3(v_roll, v_pitch, v_yaw)
					));
		
		double lpf_step;
		nh.param("lpf_step", lpf_step, 16.0);
		f_pos[0].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		f_pos[1].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		f_pos[2].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		f_ang[0].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0, true));
		f_ang[1].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0, true));
		f_ang[2].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0, true));
		
		nh.param("acc_lpf_step", lpf_step, 128.0);
		f_acc[0].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		f_acc[1].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		f_acc[2].reset(new filter(filter::FILTER_LPF, lpf_step, 0.0));
		nh.param("acc_var", params.acc_var, M_PI / 4.0); // 45 deg

		nh.param("jump_dist", params.jump_dist, 1.0);
		nh.param("jump_ang", params.jump_ang, 1.57);
		nh.param("fix_dist", params.fix_dist, 0.2);
		nh.param("fix_ang", params.fix_ang, 0.1);
		nh.param("bias_var_dist", params.bias_var_dist, 2.0);
		nh.param("bias_var_ang", params.bias_var_ang, 1.57);

		nh.param("skip_measure", params.skip_measure, 1);
		cnt_measure = 0;
		nh.param("accum_cloud", params.accum_cloud, 1);
		cnt_accum = 0;

		nh.param("match_output_dist", params.match_output_dist, 0.1);
		nh.param("unmatch_output_dist", params.unmatch_output_dist, 0.5);
		double match_output_interval_t;
		nh.param("match_output_interval_interval", match_output_interval_t, 0.2);
		params.match_output_interval.reset(new ros::Duration(match_output_interval_t));
		pub_matched = nh.advertise<sensor_msgs::PointCloud2>("matched", 2, true);
		pub_unmatched = nh.advertise<sensor_msgs::PointCloud2>("unmatched", 2, true);

		double tf_tolerance_t;
		nh.param("tf_tolerance", tf_tolerance_t, 0.05);
		params.tf_tolerance.reset(new ros::Duration(tf_tolerance_t));

		nh.param("publish_tf", publish_tf, true);
		nh.param("output_pcd", output_pcd, false);

		has_odom = has_map = false;
		match_output_last = ros::Time::now();
		localize_rate.reset(new filter(filter::FILTER_LPF, 5.0, 0.0));
		localized_last = ros::Time::now();
	}
	~mcl_3dl_node()
	{
	}
	void spin()
	{
		ros::Rate rate(10);
		ros::Time map_update_interval_start = ros::Time::now();

		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();

			ros::Time now = ros::Time::now();
			if(map_update_interval_start + *params.map_update_interval < now)
			{
				map_update_interval_start = now;
				if(has_map)
				{
					auto e = pf->expectation(1.0);
					*pc_map2 = *pc_map + *pc_update;
					pc_map2->points.erase(
							std::remove_if(pc_map2->points.begin(), pc_map2->points.end(),
								[&](const pcl::PointXYZI &p)
								{
									if(p.z - e.pos.z > params.map_clip_z_max) return true;
									if(p.z - e.pos.z < params.map_clip_z_min) return true;
									return false;
								}), pc_map2->points.end());
					pc_map2->width = 1;
					pc_map2->height = pc_map2->points.size();
					kdtree.reset(new  pcl::KdTreeFLANN<pcl::PointXYZI>);
					kdtree->setEpsilon(params.map_grid_min);
					kdtree->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep));
					kdtree->setInputCloud(pc_map2);

					sensor_msgs::PointCloud2 out;
					pcl::toROSMsg(*pc_map2, out);
					pub_mapcloud.publish(out);
				}
			}
		}
		if(output_pcd)
		{

			if(pc_all_accum)
			{
				std::cerr << "mcl_3dl: saving pcd file.";
				std::cerr << " (" << (int)pc_all_accum->points.size() << " points)" << std::endl;
				pcl::io::savePCDFileBinary("mcl_3dl.pcd", *pc_all_accum);
			}
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mcl_3dl");

	mcl_3dl_node mcl(argc, argv);
	mcl.spin();

	return 0;
}



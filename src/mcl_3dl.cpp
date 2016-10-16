#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

	
class mcl_3dl_node
{
private:
	ros::Subscriber sub_cloud;
	ros::Subscriber sub_mapcloud;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_position;
	ros::Publisher pub_particle;
	ros::Publisher pub_debug;
	ros::Publisher pub_mapcloud;

	tf::TransformListener tfl;
	tf::TransformBroadcaster tfb;

	std::shared_ptr<filter> f_pos[3]; 
	std::shared_ptr<filter> f_ang[3]; 

	class parameters
	{
	public:
		double clip_near;
		double clip_far;
		double clip_near_sq;
		double clip_far_sq;
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
		double match_dist_min;
		double match_weight;
		std::shared_ptr<ros::Duration> map_update_interval;
		int num_particles;
	} params;

	class state: public pf::particleBase<float>
	{
	public:
		vec3 pos;
		quat rot;
		struct
		{
			vec3 lin;
			vec3 ang;
		} vel;
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
		state():
			rot(0.0, 0.0, 0.0, 0.0)
		{
		};
		state(const vec3 pos, const quat rot)
		{
			this->pos = pos;
			this->rot = rot;
		};
		state(const vec3 pos, const quat rot, const vec3 lin, const vec3 ang)
		{
			this->pos = pos;
			this->rot = rot;
			this->vel.lin = lin;
			this->vel.ang = ang;
		};
		void transform(pcl::PointCloud<pcl::PointXYZ> &pc) const
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
	};
	std::shared_ptr<pf::particle_filter<state>> pf;

	class MyPointRepresentation: public pcl::PointRepresentation<pcl::PointXYZ>
	{
		using pcl::PointRepresentation<pcl::PointXYZ>::nr_dimensions_;
	public:
		MyPointRepresentation ()
		{
			nr_dimensions_ = 4;
		}

		virtual void copyToFloatArray(const pcl::PointXYZ &p, float * out) const
		{
			out[0] = p.x;
			out[1] = p.y;
			out[2] = p.z;
		}
	};
	MyPointRepresentation point_rep;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_update;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_orig;
	void cb_mapcloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		ROS_INFO("map received");
		pcl::PointCloud<pcl::PointXYZ> pc_tmp;
		pcl::fromROSMsg(*msg, pc_tmp);

		pc_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pc_map2.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pc_update.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> ds;
		ds.setInputCloud(pc_tmp.makeShared());
		ds.setLeafSize(params.map_downsample_x, params.map_downsample_y, params.map_downsample_z);
		ds.filter(*pc_map);
		pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZ>);
		frame_num = 0;
		has_map = true;
		*pc_map2 = *pc_map;
		kdtree.reset(new  pcl::KdTreeFLANN<pcl::PointXYZ>);
		kdtree->setEpsilon(params.map_grid_min / 2);
		kdtree->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep));
		kdtree->setInputCloud(pc_map2);
		kdtree_orig.reset(new  pcl::KdTreeFLANN<pcl::PointXYZ>);
		kdtree_orig->setInputCloud(pc_map);
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
					vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.0),
					quat(0.0, 0.0,
						pose.pose.orientation.z,
						pose.pose.orientation.w)
					), 
				state(
					vec3(msg->pose.covariance[0], msg->pose.covariance[6+1], 0.3),
					quat(0.0, 0.0, 0.1, 0.1)
					));
		pc_update.reset(new pcl::PointCloud<pcl::PointXYZ>);
	}

	std::map<std::string, std::string> frame_ids;

	bool update_map;

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

			if(dt > 0.05)
			{
			//	ROS_INFO("dt %0.3f", dt);
				vec3 v = odom_prev.rot.inv() * (odom.pos - odom_prev.pos);
				quat r = odom_prev.rot.inv() * odom.rot;
				r.normalize();
				std::normal_distribution<float> nd(1.0, 1.0);
				std::normal_distribution<float> nd2(1.0, 0.1);
				pf->predict([&](state &s)
						{
							s.rot.normalize();
							s.rot = (r * nd2(engine)) * s.rot;
							s.pos += s.rot * (v * nd(engine));
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local_accum;
	void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		if(!has_map) return;
		if(frames.find(msg->header.frame_id) == frames.end())
		{
			frames[msg->header.frame_id] = true;
			frames_v.push_back(msg->header.frame_id);
		}

		sensor_msgs::PointCloud2 pc_bl;
		if(!pcl_ros::transformPointCloud("base_link", *msg, pc_bl, tfl))
		{
			return;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(pc_bl, *pc_tmp);
		*pc_local_accum += *pc_tmp;

		if(frames_v[frame_num].compare(msg->header.frame_id) != 0) return;
		frame_num ++;
		if(frame_num >= frames_v.size()) frame_num = 0;

		//ROS_INFO("cloud %s  %d/%d", msg->header.frame_id.c_str(), frame_num, frames_v.size());

		if(frame_num != 0) return;

		const auto ts = std::chrono::high_resolution_clock::now();

		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> ds;
		ds.setInputCloud(pc_local_accum);
		ds.setLeafSize(params.downsample_x, params.downsample_y, params.downsample_z);
		ds.filter(*pc_local);
		pc_local_accum.reset(new pcl::PointCloud<pcl::PointXYZ>);

		std::uniform_real_distribution<float> ud(0.0, 1.0);
		pc_local->points.erase(
				std::remove_if(pc_local->points.begin(), pc_local->points.end(),
					[&](const pcl::PointXYZ &p)
					{
						if(p.x*p.x + p.y*p.y > params.clip_far_sq) return true;
						if(p.x*p.x + p.y*p.y < params.clip_near_sq) return true;
						if(p.z < 0.0) return true;
						return ud(engine) > 0.2;
					}), pc_local->points.end());
		pc_local->width = 1;
		pc_local->height = pc_local->points.size();
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_particle(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dummy(new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<int> id(1);
		std::vector<float> sqdist(1);

		const float match_dist_min = params.match_dist_min;
		const float match_weight = params.match_weight;
		pf->measure([&](const state &s)->float
				{
					pcl::PointXYZ p0 = pcl::PointXYZ(s.pos.x, s.pos.y, s.pos.z + 0.3);
					if(kdtree_orig->nearestKSearch(p0, 1, id, sqdist))
					{
						if(sqdist[0] < 0.2*0.2) return 0.0;
					}

					float score = 0;
					*pc_particle = *pc_local;
					s.transform(*pc_particle);

					size_t num = 0;
					for(auto &p: pc_particle->points)
					{
						if(kdtree->nearestKSearch(p, 1, id, sqdist))
						{
							float dist = sqdist[0];
							if(dist < 0.05 * 0.05) dist = 0.05 * 0.05;
							dist = match_dist_min - sqrtf(dist);
							if(dist < 0.0) continue;
							score += dist * match_weight;
							num ++;
						}
					}
					
					return score;
				});

		auto e = pf->max();
		e.rot.x = 0.0;
		e.rot.y = 0.0;
		e.rot.normalize();

		vec3 map_pos;
		quat map_rot;
		map_pos = e.pos - e.rot * odom.rot.inv() * odom.pos;
		map_rot = e.rot * odom.rot.inv();
		tf::StampedTransform trans;
		trans.stamp_ = ros::Time::now() + ros::Duration(0.2);
		trans.frame_id_ = frame_ids["map"];
		trans.child_frame_id_ = "odom";
		auto rpy = map_rot.get_rpy();
		rpy.x = f_ang[0]->in(rpy.x);
		rpy.y = f_ang[1]->in(rpy.y);
		rpy.z = f_ang[2]->in(rpy.z);
		map_rot.set_rpy(rpy);
		trans.setOrigin(tf::Vector3(f_pos[0]->in(map_pos.x), f_pos[1]->in(map_pos.y), f_pos[2]->in(map_pos.z)));
		trans.setRotation(tf::Quaternion(map_rot.x, map_rot.y, map_rot.z, map_rot.w));
		tfb.sendTransform(trans);

		trans.stamp_ = ros::Time::now() + ros::Duration(0.2);
		trans.frame_id_ = frame_ids["map"];
		trans.child_frame_id_ = "floor";
		trans.setOrigin(tf::Vector3(0.0, 0.0, e.pos.z));
		trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
		tfb.sendTransform(trans);

		*pc_particle = *pc_local;
		e.transform(*pc_particle);
		sensor_msgs::PointCloud pc;
		pc.header.stamp = ros::Time::now();
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
				if(update_map)
				{
					if(sqdist[0] > 0.6 * 0.6)
					{
						pc_update->insert(pc_update->end(), p);
					}
				}
			}
		}
		pub_debug.publish(pc);
			
		geometry_msgs::PoseArray pa;
		pa.header.stamp = ros::Time::now();
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
					vec3(0.1, 0.1, 0.05),
					quat(0.005, 0.005, 0.05, 0.05)
					)
				);

		const auto tnow = std::chrono::high_resolution_clock::now();
		ROS_INFO("MCL (%0.3f sec.)",
				std::chrono::duration<float>(tnow - ts).count());
	}

public:
	mcl_3dl_node(int argc, char *argv[]):
		engine(seed_gen())
	{
		ros::NodeHandle nh("~");

		sub_cloud = nh.subscribe("cloud", 20, &mcl_3dl_node::cb_cloud, this);
		sub_odom = nh.subscribe("odom", 1000, &mcl_3dl_node::cb_odom, this);
		sub_mapcloud = nh.subscribe("mapcloud", 1, &mcl_3dl_node::cb_mapcloud, this);
		sub_position = nh.subscribe("initialpose", 1, &mcl_3dl_node::cb_position, this);
		pub_particle = nh.advertise<geometry_msgs::PoseArray>("particles", 1, true);
		pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 5, true);
		pub_mapcloud = nh.advertise<sensor_msgs::PointCloud2>("updated_map", 1, true);

		nh.param("update_map", update_map, false);
		nh.param("map_frame", frame_ids["map"], std::string("map_ground"));
		nh.param("clip_near", params.clip_near, 0.5);
		nh.param("clip_far", params.clip_far, 8.0);
		params.clip_near_sq = pow(params.clip_near, 2.0);
		params.clip_far_sq = pow(params.clip_far, 2.0);
		nh.param("map_downsample_x", params.map_downsample_x, 0.1);
		nh.param("map_downsample_y", params.map_downsample_y, 0.1);
		nh.param("map_downsample_z", params.map_downsample_z, 0.1);
		nh.param("downsample_x", params.downsample_x, 0.2);
		nh.param("downsample_y", params.downsample_y, 0.2);
		nh.param("downsample_z", params.downsample_z, 0.1);
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
		float weight_f[3];
		nh.param("dist_weight_x", weight[0], 1.0);
		nh.param("dist_weight_y", weight[1], 1.0);
		nh.param("dist_weight_z", weight[2], 5.0);
		for(size_t i = 0; i < 3; i ++) weight_f[i] = weight[i];
		point_rep.setRescaleValues(weight_f);

		nh.param("num_particles", params.num_particles, 128);
		pf.reset(new pf::particle_filter<state>(params.num_particles));

		pf->init(
				state(
					vec3(-0.5, 1.0, 3.0),
					quat(0.0, 0.0, 0.0, 1.0)
					), 
				state(
					vec3(0.8, 0.8, 0.4),
					quat(0.0, 0.0, 0.2, 0.2)
					));
		
		f_pos[0].reset(new filter(filter::FILTER_LPF, 5.0, 0.0));
		f_pos[1].reset(new filter(filter::FILTER_LPF, 5.0, 0.0));
		f_pos[2].reset(new filter(filter::FILTER_LPF, 5.0, 0.0));
		f_ang[0].reset(new filter(filter::FILTER_LPF, 5.0, 0.0, true));
		f_ang[1].reset(new filter(filter::FILTER_LPF, 5.0, 0.0, true));
		f_ang[2].reset(new filter(filter::FILTER_LPF, 5.0, 0.0, true));

		has_odom = has_map = false;
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
					*pc_map2 = *pc_map;
					if(update_map)
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::VoxelGrid<pcl::PointXYZ> ds;
						ds.setInputCloud(pc_update);
						ds.setLeafSize(params.update_downsample_x, 
								params.update_downsample_y, 
								params.update_downsample_z);
						ds.filter(*pc_tmp);
						*pc_update = *pc_tmp;
						*pc_map2 += *pc_update;
					}
					pc_map2->points.erase(
							std::remove_if(pc_map2->points.begin(), pc_map2->points.end(),
								[&](const pcl::PointXYZ &p)
								{
								if(p.z - e.pos.z > 2.0) return true;
								if(p.z - e.pos.z < -0.2) return true;
								return false;
								}), pc_map2->points.end());
					pc_map2->width = 1;
					pc_map2->height = pc_map2->points.size();
					kdtree.reset(new  pcl::KdTreeFLANN<pcl::PointXYZ>);
					kdtree->setEpsilon(params.map_grid_min);
					kdtree->setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_rep));
					kdtree->setInputCloud(pc_map2);

					sensor_msgs::PointCloud2 out;
					pcl::toROSMsg(*pc_map2, out);
					pub_mapcloud.publish(out);
				}
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



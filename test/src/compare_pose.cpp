#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <gtest/gtest.h>

TEST(compare_pose, compare_tf)
{
	ros::NodeHandle nh("~");
	tf::TransformListener tfl;

	size_t cnt = 0;
	const size_t cnt_max = 4;
	size_t tf_ex_cnt = 0;

	bool shutdown(false);

	const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb_pose = 
		[&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)->void{
			geometry_msgs::PoseStamped pose;
			try
			{
				geometry_msgs::PoseStamped pose_bl;
				pose_bl.header.frame_id = "base_link";
				pose_bl.header.stamp = msg->header.stamp;
				pose_bl.pose.orientation.w = 1.0;
				tfl.waitForTransform("map", pose_bl.header.frame_id, 
						pose_bl.header.stamp, ros::Duration(0.1));
				tfl.transformPose("map", pose_bl, pose);
			}
			catch(tf::TransformException &e)
			{
				tf_ex_cnt ++;
				return;
			}
			float x_error = pose.pose.position.x - msg->pose.pose.position.x;
			float y_error = pose.pose.position.y - msg->pose.pose.position.y;
			float z_error = pose.pose.position.z - msg->pose.pose.position.z;
			float error = sqrtf(powf(x_error, 2.0) + powf(y_error, 2.0) + powf(z_error, 2.0));

			fprintf(stderr, "compare_tf[%d/%d]:\n",
					(int)cnt, (int)cnt_max);
			fprintf(stderr, "  error=%0.3f\n", error);

			cnt ++;
			if(cnt >= cnt_max) shutdown = true;

			ASSERT_FALSE(error > 0.01)
				<< "tf output diverges from amcl_pose.";
		};
	ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 1, cb_pose);
	
	ros::Rate wait(1);

	while(ros::ok() && !shutdown)
	{
		ros::spinOnce();
		wait.sleep();
	}
	
	ASSERT_FALSE(tf_ex_cnt > 1)
		<< "tf exception occures more than once.";
}

TEST(compare_pose, compare)
{
	ros::NodeHandle nh("~");
	
	bool shutdown(false);

	nav_msgs::Path path;
	size_t i_path;

	double error_limit;
	nh.param("error_limit", error_limit, 0.3);

	const boost::function<void(const nav_msgs::Path::ConstPtr&)> cb_path = 
		[&](const nav_msgs::Path::ConstPtr &msg)->void{
			path = *msg;
			i_path = 0;
			fprintf(stderr, "compare_pose: reference received\n");
		};
	const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb_pose = 
		[&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)->void{
			if(path.poses.size() > 0 && path.poses[i_path].header.stamp < ros::Time::now())
			{
				float x_error = path.poses[i_path].pose.position.x - msg->pose.pose.position.x;
				float y_error = path.poses[i_path].pose.position.y - msg->pose.pose.position.y;
				float z_error = path.poses[i_path].pose.position.z - msg->pose.pose.position.z;
				float error = sqrtf(powf(x_error, 2.0) + powf(y_error, 2.0) + powf(z_error, 2.0));
				float x_sigma = sqrtf(msg->pose.covariance[0*6+0]);
				float y_sigma = sqrtf(msg->pose.covariance[1*6+1]);
				float z_sigma = sqrtf(msg->pose.covariance[2*6+2]);

				fprintf(stderr, "compare_pose[%d/%d]:\n",
						(int)i_path, (int)path.poses.size());
				fprintf(stderr, "  position error/limit=%0.3f/%0.3f\n", error, error_limit);
				fprintf(stderr, "  x error/3sigma=%0.3f/%0.3f\n", x_error, x_sigma * 3.0);
				fprintf(stderr, "  y error/3sigma=%0.3f/%0.3f\n", y_error, y_sigma * 3.0);
				fprintf(stderr, "  z error/3sigma=%0.3f/%0.3f\n", z_error, z_sigma * 3.0);

				i_path ++;
				if(i_path >= path.poses.size()) shutdown = true;

				ASSERT_FALSE(error > error_limit)
					<< "Position error is larger then expected.";
				ASSERT_FALSE(fabs(x_error) > x_sigma * 3.0)
					<< "Estimated variance is too small to continue tracking. (x)";
				ASSERT_FALSE(fabs(y_error) > y_sigma * 3.0)
					<< "Estimated variance is too small to continue tracking. (y)";
				ASSERT_FALSE(fabs(z_error) > z_sigma * 3.0)
					<< "Estimated variance is too small to continue tracking. (z)";
			}
		};

	ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 1, cb_pose);
	ros::Subscriber sub_path = nh.subscribe("poses_ref", 1, cb_path);

	ros::Rate wait(10);

	while(ros::ok() && !shutdown)
	{
		ros::spinOnce();
		wait.sleep();
	}
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "compare_pose");

	return RUN_ALL_TESTS();
}


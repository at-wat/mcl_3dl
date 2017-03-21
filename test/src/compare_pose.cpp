#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gtest/gtest.h>

TEST(compare_pose, compare)
{
	ros::NodeHandle nh("~");
	
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
				if(i_path >= path.poses.size()) ros::shutdown();

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

	while(ros::ok())
	{
		ros::spinOnce();
		wait.sleep();
	}
	fprintf(stderr, "compare_pose finished\n");
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "compare_pose");

	return RUN_ALL_TESTS();
}


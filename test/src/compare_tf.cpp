#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <gtest/gtest.h>

TEST(compare_tf, compare)
{
	ros::NodeHandle nh("~");
	tf::TransformListener tfl;

	int cnt = 0;
	int cnt_max;
	nh.param("cnt_max", cnt_max, 10);
	size_t tf_ex_cnt = 0;

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
			if(cnt >= cnt_max) ros::shutdown();

			ASSERT_FALSE(error > 0.05)
				<< "tf output diverges from amcl_pose.";
		};
	ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 1, cb_pose);
	
	ros::Rate wait(1);

	while(ros::ok())
	{
		ros::spinOnce();
		wait.sleep();
	}
	
	ASSERT_FALSE(tf_ex_cnt > 1)
		<< "tf exception occures more than once.";
	
	fprintf(stderr, "compare_tf finished\n");
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "compare_tf");

	return RUN_ALL_TESTS();
}


/*
 * TrajectoryControlActionServer.h
 *
 *  Created on: Jan 13, 2014
 *      Author: alcor
 */

#ifndef PATH_FOLLOWING_H_
#define PATH_FOLLOWING_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <path_following_msgs/WaypointFollowAction.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nifti_robot_driver_msgs/Tracks.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
	T v;
	if (n.getParam(name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}


class PathFollowing
{
protected:
	ros::NodeHandle node;
	ros::NodeHandle n_;
	std::string action_name;
	actionlib::SimpleActionServer<path_following_msgs::WaypointFollowAction> as;

	path_following_msgs::WaypointFollowFeedback feedback;
	path_following_msgs::WaypointFollowResult result;

	std::string global_frame_id;
	std::string robot_frame_id;
	


	double displacement;
	double vel_reference;

	tf::TransformListener tf_;


	std::string tracks_vel_cmd_topic;
	std::string cmd_vel_topic;
	ros::Publisher tracks_vel_cmd_pub;
	ros::Publisher cmd_vel_pub;
	
	std::string robot_pose_topic;
	ros::Subscriber robot_pose_sub;


	double linear_vel;
	double angular_vel;
	double robot_width;
	double k1;
	double k2;
	double frequency;
	
	geometry_msgs::PoseStamped robot_pose;
	//geometry_msgs::PoseStamped prev_target_pose;
	double threshold;
	bool load;


public:
	PathFollowing(std::string);
	virtual ~PathFollowing();

	void getRobotCommands(geometry_msgs::PoseStamped, double&, double&, double&);
	void getTracksVelCmd(double,double,nifti_robot_driver_msgs::Tracks& tracks_cmd, geometry_msgs::Twist& cmd_vel_msg);
	void executeCallback(const path_following_msgs::WaypointFollowGoalConstPtr&);
	void robotPoseCallBack(const geometry_msgs::PoseStampedConstPtr&);
};


#endif /* PATH_FOLLOWING_H_ */

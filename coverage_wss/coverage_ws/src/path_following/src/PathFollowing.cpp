/*
 * TrajectoryControlActionServer.cpp
 *
 *  Created on: Jan 13, 2014
 *      Author: alcor
 */

#include <math.h>
#include <path_following/PathFollowing.h>

PathFollowing::PathFollowing(std::string name) :
n_("~"),
action_name(name),
as(node, name, boost::bind(&PathFollowing::executeCallback, this, _1), false),
linear_vel(0),
angular_vel(0),
robot_width(0.6),
load(0)
{

    global_frame_id = getParam<std::string>(n_, "global_frame_id", "/map");
    robot_frame_id = getParam<std::string>(n_, "robot_frame_id", "/base_link");

    displacement = getParam<double>(n_, "displacement", 0.2);
    vel_reference = getParam<double>(n_, "vel_reference", 0.15);
    tracks_vel_cmd_topic = getParam<std::string>(n_, "tracks_vel_cmd_topic", "/tracks_vel_cmd");
    cmd_vel_topic = getParam<std::string>(n_, "cmd_vel_topic", "/cmd_vel");
    k1 = getParam<double>(n_, "gain_k1", 0.3);
    k2 = getParam<double>(n_, "gain_k2", 0.3);
    frequency = getParam<double>(n_, "frequency", 1);
    robot_pose_topic = getParam<std::string>(n_, "robot_pose_topic", "/pose");
    threshold = getParam<double>(n_, "distance_threshold", 0.1);


    tracks_vel_cmd_pub = node.advertise<nifti_robot_driver_msgs::Tracks>(tracks_vel_cmd_topic, 1);
    //cmd_vel_pub = node.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    robot_pose_sub = node.subscribe(robot_pose_topic, 1, &PathFollowing::robotPoseCallBack, this);

    as.start();

}

PathFollowing::~PathFollowing()
{
}

void PathFollowing::robotPoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg)
{
    robot_pose.header.frame_id = msg->header.frame_id;
    robot_pose.header.stamp = msg->header.stamp;
    robot_pose.pose.position.x = msg->pose.position.x;
    robot_pose.pose.position.y = msg->pose.position.y;
    robot_pose.pose.position.z = msg->pose.position.z;
    robot_pose.pose.orientation.x = msg->pose.orientation.x;
    robot_pose.pose.orientation.y = msg->pose.orientation.y;
    robot_pose.pose.orientation.z = msg->pose.orientation.z;
    robot_pose.pose.orientation.w = msg->pose.orientation.w;
}

void PathFollowing::getRobotCommands(geometry_msgs::PoseStamped target_pose, double& linear_vel, double& angular_vel, double& error_dist)
{

    double robot_yaw = tf::getYaw(robot_pose.pose.orientation);
    double a = cos(robot_yaw);
    double b = sin(robot_yaw);
    double c = -sin(robot_yaw) / displacement;
    double d = cos(robot_yaw) / displacement;

    // point towards the target 
    double target_yaw = atan2(target_pose.pose.position.y - (robot_pose.pose.position.y + b * displacement), target_pose.pose.position.x - (robot_pose.pose.position.x + a * displacement));

    double u1 = vel_reference * cos(target_yaw) + k1 * (target_pose.pose.position.x - (robot_pose.pose.position.x + a * displacement));
    double u2 = vel_reference * sin(target_yaw) + k2 * (target_pose.pose.position.y - (robot_pose.pose.position.y + b * displacement));
    //std::cout<< "vel_reference * cos(target_yaw)" << vel_reference * cos(target_yaw) << " 2x: " << k1 * (target_pose.pose.position.x - (robot_pose.pose.position.x + a*displacement)) << std::endl;
    //std::cout<< "vel_reference * sin(target_yaw)" << vel_reference * sin(target_yaw) << " 2y: " << k2 * (target_pose.pose.position.y - (robot_pose.pose.position.y + b*displacement)) << std::endl;

    //double u1 = k1 * (poseB.position.x - robot_pose.getOrigin().getX());
    //double u2 = k2 * (poseB.position.y - robot_pose.getOrigin().getY());

    linear_vel = a * u1 + b * u2;
    angular_vel = c * u1 + d * u2;

    error_dist = sqrt(pow(target_pose.pose.position.x - (robot_pose.pose.position.x + a * displacement), 2) + pow(target_pose.pose.position.y - (robot_pose.pose.position.y + b * displacement), 2));
    //std::cout << "Distance error " << error_dist << std::endl;
}

void PathFollowing::getTracksVelCmd(double linear_vel, double angular_vel, nifti_robot_driver_msgs::Tracks& tracks_cmd, geometry_msgs::Twist& cmd_vel_msg)
{
    double d = robot_width / 2;

    tracks_cmd.left = linear_vel - (d * angular_vel);
    tracks_cmd.right = linear_vel + (d * angular_vel);
    cmd_vel_msg.linear.x = linear_vel;
    cmd_vel_msg.angular.z = angular_vel;
    /*
            if(tracks_cmd.left < -0.6)
            {
                    tracks_cmd.left = -0.6;
            }
            if(tracks_cmd.left > 0.6)
            {
                    tracks_cmd.left = 0.6;
            }
            if(tracks_cmd.right < -0.6)
            {
                    tracks_cmd.right = -0.6;
            }
            if(tracks_cmd.right > 0.6)
            {
                    tracks_cmd.right = 0.6;
            }
     */
}

void PathFollowing::executeCallback(const path_following_msgs::WaypointFollowGoalConstPtr &goal)
{
    nifti_robot_driver_msgs::Tracks tracks_cmd;
    geometry_msgs::Twist cmd_vel_msg;

    geometry_msgs::PoseStamped target_pose = goal->target_pose;

    double dist_error = 0;
    //std::cout << "Distance error " << dist_error << std::endl;

    bool done = false;
    ros::Rate rate(frequency);

    while (ros::ok() && !done)
    {
        //		if(load == 0)
        //		{
        //			load = 1;
        //			prev_target_pose = target_pose;
        //			done = true;
        //		}
        //		else
        {
            //ROS_INFO(" ---- cycle step ----  ");
            if (as.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name.c_str());
                getTracksVelCmd(0, 0, tracks_cmd, cmd_vel_msg);
                tracks_vel_cmd_pub.publish(tracks_cmd);
                //cmd_vel_pub.publish(cmd_vel_msg);
                as.setPreempted();
                break;
            }
            else
            {

                getRobotCommands(target_pose, linear_vel, angular_vel, dist_error);
                getTracksVelCmd(linear_vel, angular_vel, tracks_cmd, cmd_vel_msg);
                //ROS_INFO("Robot tracks vel computed");
                feedback.tracks_cmd.left = tracks_cmd.left;
                feedback.tracks_cmd.right = tracks_cmd.right;
                feedback.distance_error.x = target_pose.pose.position.x - robot_pose.pose.position.x;
                feedback.distance_error.y = target_pose.pose.position.y - robot_pose.pose.position.y;
                //as.publishFeedback(feedback);
                tracks_vel_cmd_pub.publish(tracks_cmd);
                //cmd_vel_pub.publish(cmd_vel_msg);
                //ROS_INFO("Robot tracks vel published");
                rate.sleep();
                if (dist_error < threshold)
                {
                    done = true;
                    //prev_target_pose = target_pose;
                }
            }
        }

    }
    if (done)
    {
        result.done = true;
        getTracksVelCmd(0, 0, tracks_cmd, cmd_vel_msg);
        tracks_vel_cmd_pub.publish(tracks_cmd);
        //cmd_vel_pub.publish(cmd_vel_msg);
        rate.sleep();
        ROS_INFO("%s: Succeeded", action_name.c_str());
        as.setSucceeded(result);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    PathFollowing action_server(ros::this_node::getName());

    ros::spin();

    return 0;
}

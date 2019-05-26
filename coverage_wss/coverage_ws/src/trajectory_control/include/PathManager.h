
#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <boost/thread/recursive_mutex.hpp>

#include <path_planner/KdTreeFLANN.h>

#include "SignalUtils.h"

///	\class PathManager
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class PathManager
{
public:
 
    typedef pcl::PointXYZL PointNode;
        
    static const double kDistanceToRecomputeIndex;
    
public:
    
    PathManager();
    virtual ~PathManager();

    /// Init
    virtual void init(double Ts, double vel, const nav_msgs::Path& path_in);

    /// basic step for generating next ref point 
    /// return true when done 
    virtual bool step();
    
    
    void sendMarkers(); 

public: // setters 
    
    void setTs(double Ts) { d_Ts_ = Ts; }
    void setVel(double vel) { d_vel_lin_ = vel; }
    
    void setRobotPosition(double x, double y, double z);
    
public: // getters
    
    nav_msgs::Path& getPathOut() {return path_out_; }
    
    double getEstimatedDistance() const { return d_estimated_distance_;}
    double getEstimatedTime() const { return d_estimated_time_;}
    
    bool isPathEnd() const {return b_end_; }
    
    const geometry_msgs::PoseStamped& getCurrentPose() const {return current_pose_;}
    
    const geometry_msgs::PoseStamped& getFinalPose() const {return final_pose_;}
    
    double getAngularVel() const {return d_vel_ang_;}
    
    double getCurrentYaw() const {return d_yaw_last_;}
    
protected:
    
    boost::recursive_mutex mutex_;    
    
    bool b_init_; // has object been initialized?
    bool b_end_; // is path ended? 
    bool b_jump_point_; // 
    
    double d_Ts_; // nominal time step
    double d_vel_lin_; // nominal linear velocity 
    double d_vel_ang_; // reference angular velocity (approximated numerically)  
    double d_yaw_last_; // last yaw
    
    double d_estimated_time_; // nominal estimated time 
    double d_estimated_distance_; // nominal estimated distance 
   
    nav_msgs::Path path_in_; // input path
    
    nav_msgs::Path path_out_; // output path 
    
    geometry_msgs::PoseStamped current_pose_; // current pose 
    geometry_msgs::PoseStamped jumped_pose_; // jumped pose during the resampling  
    geometry_msgs::PoseStamped final_pose_; // final pose 
    
    PointNode robot_position_;  
    
    size_t i_index_; // index of the current pose 
    
    double d_step_offset_; // step offset used for generating new points on the trajectory 
    
protected:
    
    //Ros node handle
    ros::NodeHandle node_;      
};


///	\class PathManagerKdt
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class PathManagerKdt: public PathManager
{
public:
    

    typedef pcl::PointCloud<PointNode> PointCloudNodes;
    typedef pp::KdTreeFLANN<PointNode> KdTreeNodes;  
    
public:
    
    PathManagerKdt(){}
    
    /// Init
    void init(double Ts, double vel, const nav_msgs::Path& path_in);

    /// basic step for generating next ref point 
    /// return true when done 
    bool step();
    
protected: 
      
    PointCloudNodes plc_points_;
    KdTreeNodes     kdtree_points_;         
};

#endif

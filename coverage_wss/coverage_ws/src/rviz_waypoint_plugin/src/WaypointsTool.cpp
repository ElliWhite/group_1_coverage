/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
#include<ros/console.h>
#include<rviz/viewport_mouse_event.h>
#include<rviz/visualization_manager.h>
#include<rviz/geometry.h>
#include<rviz/mesh_loader.h>
#include "WaypointsTool.h"

namespace rviz_waypoint_plugin
{

/* BEGIN_TUTORIAL
 * Construction and destruction
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * The constructor must have no arguments, so we can't give the
 * constructor the parameters it needs to fully initialize.
 *
 * Here we set the "shortcut_key_" member variable defined in the
 * superclass to declare which key will activate the tool. */
WaypointsTool::WaypointsTool()
{
    shortcut_key_ = 'm';

    // point cloud update
    pcl_sub_= node.subscribe<PointCloud>("/point_map",1,&WaypointsTool::cloudCallback,this);

    // internal counters
    planner_count_=1; // handles planning task ordering
    markers_count_=1; // handles markers naming

    // markers frame and publisher
    markers_frame_id_="/map";
	task_append_pub_=node.advertise<rviz_waypoint_plugin::PlanningTask>("/planner/task/append", 1);
	task_remove_pub_=node.advertise<rviz_waypoint_plugin::PlanningTask>("/planner/task/remove", 1);

    // right-click menu for 'interactive' markers (those orange)
	markers_interactive_menu_.insert("Append planning task",boost::bind(&WaypointsTool::markerInteractiveCallback,this,_1));
	markers_interactive_menu_.insert("Remove this waypoint",boost::bind(&WaypointsTool::markerInteractiveCallback,this,_1));
	markers_interactive_menu_.insert("Remove all waypoints",boost::bind(&WaypointsTool::markerInteractiveCallback,this,_1));

    // right-click menu for 'static' markers (those gray)
	markers_static_menu_.insert("Remove/stop this planning task",boost::bind(&WaypointsTool::markerStaticCallback,this,_1));
	markers_static_menu_.insert("Remove/stop all planning tasks",boost::bind(&WaypointsTool::markerStaticCallback,this,_1));

    // marker's server initialization
	markers_server_.reset( new interactive_markers::InteractiveMarkerServer("/planner/waypoints/server","",false) );
    markers_server_->clear();
    markers_server_->applyChanges();

    // load mesh for 'interactive' markers
    resource_interactive_ = "package://rviz_waypoint_plugin/mesh/barrier_interactive.dae";
    if( rviz::loadMeshFromResource( resource_interactive_ ).isNull() )
    {
        ROS_ERROR( "WaypointsTool: failed to load model resource '%s'.", resource_interactive_.c_str() );
        return;
    }

    // load mesh for 'static' markers
    resource_static_ = "package://rviz_waypoint_plugin/mesh/barrier_static.dae";
    if( rviz::loadMeshFromResource( resource_static_ ).isNull() )
    {
        ROS_ERROR( "WaypointsTool: failed to load model resource '%s'.", resource_static_.c_str() );
        return;
    }
}

/* The destructor destroys the Ogre scene nodes for the flags so they
 * disappear from the 3D scene.  The destructor for a Tool subclass is
 * only called when the tool is removed from the toolbar with the "-"
 * button. */
WaypointsTool::~WaypointsTool()
{
    // remove all markers from the server
    markers_server_->clear();
    markers_server_->applyChanges();

    // reset the sever
	markers_server_.reset();
}

void WaypointsTool::cloudCallback(const PointCloud::ConstPtr& msg)
{
    // get last point cloud
    pcl::copyPointCloud(*msg,pcd_);
}

/* Handling mouse events
 * ^^^^^^^^^^^^^^^^^^^^^
 *
 * processMouseEvent() is sort of the main function of a Tool, because
 * mouse interactions are the point of Tools.
 *
 * We use the utility function rviz::getPointOnPlaneFromWindowXY() to
 * see where on the ground plane the user's mouse is pointing, then
 * move the moving flag to that point and update the VectorProperty.
 *
 * If this mouse event was a left button press, we want to save the
 * current flag location.  Therefore we make a new flag at the same
 * place and drop the pointer to the VectorProperty.  Dropping the
 * pointer means when the tool is deactivated the VectorProperty won't
 * be deleted, which is what we want. */
int WaypointsTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    // update viewport for projection of markers over the point cloud
    // (I believe that this should be done only once)
    viewport_=event.viewport;

    // if the user pressed the left button of the mouse/touchpad, then
    if( event.leftDown() )
    {
        // find point (of the cloud) with the nearest projection to the pointer
        double distance=1e9;
        Ogre::Vector3 position(0,0,0);
        for( size_t i = 0; i < pcd_.size(); i++ )
        {
            Ogre::Vector3 P(pcd_.points[i].x,pcd_.points[i].y,pcd_.points[i].z);
            Ogre::Vector2 p=rviz::project3DPointToViewportXY(viewport_,P);

            double d=sqrt(pow(event.x-p.x,2)+pow(event.y-p.y,2));
            if( distance>d )
            {
                position=P;
                distance=d;
            }
        }

        // create the marker at the above computed position
        markerAdd( position );

        return Render | Finished;
    }

    return Render;

}

void WaypointsTool::markerAdd( const Ogre::Vector3& position )
{
    visualization_msgs::InteractiveMarker marker;

    // header
	marker.header.frame_id = markers_frame_id_;
	marker.header.stamp = ros::Time::now();

    // position
	marker.pose.position.x = position.x;
	marker.pose.position.y = position.y;
	marker.pose.position.z = position.z;
	marker.scale = 1;

    // name and description
	std::ostringstream ss;
	ss << markers_count_++;
	marker.name = ss.str();
	marker.description = ss.str();

    // mesh resource
    visualization_msgs::Marker shape;
	shape.type = visualization_msgs::Marker::MESH_RESOURCE;
    shape.mesh_resource = resource_interactive_;
    shape.mesh_use_embedded_materials=true;

    // interaction control
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    control.always_visible = true;
    control.markers.push_back(shape);
    marker.controls.push_back(control);

    // insert marker on the server
	markers_server_->insert(marker);
	markers_server_->setCallback(marker.name, boost::bind(&WaypointsTool::markerInteractiveCallback,this,_1));
	markers_interactive_menu_.apply(*markers_server_, marker.name);
	markers_server_->applyChanges();
}

void WaypointsTool::markerUpdate( const std::string& name, const geometry_msgs::Point& reference )
{
    visualization_msgs::InteractiveMarker marker;

    // get the marker
    if( !markers_server_->get(name,marker) )
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!",name.c_str()); 
        return;
    }

    // if the marker is static, the do nothing
    if( visualization_msgs::InteractiveMarkerControl::MENU==marker.controls[0].interaction_mode )
    {
        return;
    }

    // obtain pointer coordinates
    Ogre::Vector3 position=Ogre::Vector3(reference.x,reference.y,reference.z);
    Ogre::Vector2 pointer=rviz::project3DPointToViewportXY(viewport_,position);

    // find point (of the cloud) with the nearest projection of the pointer
    double distance=1e9;
    for( size_t i = 0; i < pcd_.size(); i++ )
    {
        Ogre::Vector3 P(pcd_.points[i].x,pcd_.points[i].y,pcd_.points[i].z);
        Ogre::Vector2 p=rviz::project3DPointToViewportXY(viewport_,P);

        double d=sqrt(pow(pointer.x-p.x,2)+pow(pointer.y-p.y,2));
        if( distance>d )
        {
            position=P;
            distance=d;
        }
    }

    // update the marker position
	marker.pose.position.x = position.x;
	marker.pose.position.y = position.y;
	marker.pose.position.z = position.z;

    // insert (update) the marker on the server
	markers_server_->insert(marker);
	markers_server_->applyChanges();
}

void WaypointsTool::markerClearAll( void )
{
   visualization_msgs::InteractiveMarker marker;

    // for each counted marker (or waypoint)
    for( unsigned long i=1; markers_count_>i; i++ )
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if( markers_server_->get(ss.str(),marker) )
        {
            // verify that the marker is not 'static'
            // (that is, has not been published in some planning task)
            if( visualization_msgs::InteractiveMarkerControl::MENU!=marker.controls[0].interaction_mode )
            {
                // in this case, erase (remove) marker from the server
                markers_server_->erase(marker.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

void WaypointsTool::taskAppend( const ros::Time& stamp )
{
    geometry_msgs::Pose marker_msg;
    rviz_waypoint_plugin::PlanningTask task_msg;
    visualization_msgs::InteractiveMarker marker;

    std::ostringstream ss;
    ss << "Task" << planner_count_++;
    task_msg.name=ss.str();
    task_msg.header.stamp=stamp;
    task_msg.header.frame_id=markers_frame_id_;

    // for each counted marker (or waypoint)
    for( unsigned long i=1; markers_count_>i; i++ )
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if( markers_server_->get(ss.str(),marker) )
        {
            // verify that the marker is not 'static'
            // (that is, has not been published in some planning task)
            if( visualization_msgs::InteractiveMarkerControl::MENU!=marker.controls[0].interaction_mode )
            {
                // add the waypoint (marker position) to the task message
                task_msg.waypoints.push_back(marker.pose.position);

                // convert to a non-interactive marker one
                marker.controls[0].interaction_mode=visualization_msgs::InteractiveMarkerControl::MENU;
                marker.controls[0].markers[0].mesh_resource=resource_static_;

                // change marker description to identify the planning task
                // (which is based on the planner_count_)
                marker.description=task_msg.name;

                // insert (update) marker in the server
                markers_server_->insert(marker);
                markers_static_menu_.apply(*markers_server_, marker.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();

    // append the planning task to the planner node queue
    task_append_pub_.publish(task_msg);
}

void WaypointsTool::markerInteractiveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    switch ( feedback->event_type )
    {
        //case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //    break;

        // menu selection
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            #ifdef WAYPOINT_VERBOSE
                ROS_INFO(
                    "marker %s: MENU_SELECT item id %d",
                    feedback->marker_name.c_str(),feedback->menu_entry_id);
            #endif
            switch( feedback->menu_entry_id )
            {
                case 1: // Send path to planner
                    taskAppend(feedback->header.stamp);
                    break;

                case 2: // Remove this (interactive) waypoint
                    markers_server_->erase(feedback->marker_name);
                    markers_server_->applyChanges();
                    break;

                case 3: // Remove all (interactive) waypoints
                    markerClearAll();
                    break;
            }
            break;

        //case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        //    break;

        // projects the marker to the point cloud
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            #ifdef WAYPOINT_VERBOSE
                ROS_INFO(
                    "marker %s: MOUSE_UP at (%.2f,%.2f,%.2f) in frame /%s",
                    feedback->marker_name.c_str(),
                    feedback->mouse_point.x,feedback->mouse_point.y,feedback->mouse_point.z,
                    feedback->header.frame_id.c_str());
            #endif
            markerUpdate(feedback->marker_name,feedback->mouse_point);
            break;
    }
}

void WaypointsTool::taskRemove(const std::string name)
{
    visualization_msgs::InteractiveMarker aux;
    visualization_msgs::InteractiveMarker marker;
    rviz_waypoint_plugin::PlanningTask task_msg;

    if( !markers_server_->get(name,marker) )
    {
        ROS_ERROR("Interactive marker '%s' does not exist, but produces feedback!",name.c_str()); 
        return;
    }

    // remove/stop the planning task from the planner node queue
    task_msg.name=marker.description;
    task_msg.header.stamp=ros::Time::now();
    task_msg.header.frame_id=markers_frame_id_;
    task_remove_pub_.publish(task_msg);

    // for each counted marker (or waypoint)
    for( unsigned long i=1; markers_count_>i; i++ )
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if( markers_server_->get(ss.str(),aux) )
        {
            // verify that the marker belongs to the same planning task of the reference marker
            if( !marker.description.compare(aux.description) )
            {
                // in this case, erase (remove) marker from the server
                markers_server_->erase(aux.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

void WaypointsTool::taskRemoveAll( void )
{
    visualization_msgs::InteractiveMarker marker;
    rviz_waypoint_plugin::PlanningTask task_msg;

    // remove/stop all planning tasks from the planner node queue
    task_msg.name="ALL";
    task_msg.header.stamp=ros::Time::now();
    task_msg.header.frame_id=markers_frame_id_;
    task_remove_pub_.publish(task_msg);

    // for each counted marker (or waypoint)
    for( unsigned long i=1; markers_count_>i; i++ )
    {
        std::ostringstream ss;
        ss << i;

        // find the existing ones. if marker '#i' exists, then
        if( markers_server_->get(ss.str(),marker) )
        {
            // verify that the marker is 'static'
            // (that is, has been published in some planning task)
            if( visualization_msgs::InteractiveMarkerControl::MENU==marker.controls[0].interaction_mode )
            {
                // in this case, erase (remove) marker from the server
                markers_server_->erase(marker.name);
                markers_static_menu_.apply(*markers_server_, marker.name);
            }
        }
    }

    // update markers states
    markers_server_->applyChanges();
}

void WaypointsTool::markerStaticCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    switch ( feedback->event_type )
    {
        //case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //    break;

        // menu selection
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            #ifdef WAYPOINT_VERBOSE
                ROS_INFO(
                    "marker %s: MENU_SELECT item id %d",
                    feedback->marker_name.c_str(),feedback->menu_entry_id);
            #endif
            switch( feedback->menu_entry_id )
            {
                case 1: // remove/stop the given task
                    taskRemove(feedback->marker_name);
                    break;

                case 2: // remove/stop the given task
                    taskRemoveAll();
                    break;
            }
            break;

        //case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        //    break;

        //case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        //    break;
    }
}

/* End of .cpp file
 * ^^^^^^^^^^^^^^^^
 *
 * At the end of every plugin class implementation, we end the
 * namespace and then tell pluginlib about the class.  It is important
 * to do this in global scope, outside our package's namespace. */
} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_waypoint_plugin::WaypointsTool,rviz::Tool )
// END_TUTORIAL

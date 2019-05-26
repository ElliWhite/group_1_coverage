/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef FLAG_TOOL_3D_H
#define FLAG_TOOL_3D_H

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/tool.h>
#include <interactive_markers/interactive_marker_server.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tuple>
#include <vector>

#include "Flag3DMarker.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace Ogre
{
    class SceneNode;
    class Vector3;
}

namespace rviz
{
    class VectorProperty;
    class VisualizationManager;
    class ViewportMouseEvent;
}

namespace rviz_waypoint_plugin
{
    // BEGIN_TUTORIAL
    // Here we declare our new subclass of rviz::Tool.  Every tool
    // which can be added to the tool bar is a subclass of
    // rviz::Tool.
    class FlagTool3D: public rviz::Tool
    {
        Q_OBJECT
        public:
            FlagTool3D();
            ~FlagTool3D();

            virtual void onInitialize();
            virtual void activate();
            virtual void deactivate();

            virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

            virtual void load( const rviz::Config& config );
            virtual void save( rviz::Config config ) const;

            void callback(const PointCloud::ConstPtr& msg);

        protected:
            void makeFlag( const Ogre::Vector3& position );
            double computeDistance( Ogre::Vector2& a, Ogre::Vector2& b );

        protected:
            std::vector<Ogre::SceneNode*> flag_nodes_;
            std::vector<Flag3DMarker> flag_markers_;
            Ogre::SceneNode* moving_flag_node_;
            std::string flag_resource_;
            rviz::VectorProperty* current_flag_property_;

            ros::NodeHandle node;
            ros::Publisher pub;
            geometry_msgs::PoseArray wplist;

            ros::Subscriber sub;
            PointCloud pcd;

        // interactive markers stuff
        protected:
            void markerAdd( const Ogre::Vector3& position );
            void markerCreate( InteractiveMarker& position );
            void markerUpdate( const std::string& name, const geometry_msgs::Point& reference );
            void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

            int markers_count_;
            interactive_markers::MenuHandler markers_menu_handler_;
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> markers_server_;

        // bridge between interactive markers and the tool
        protected:
            Ogre::Viewport *viewport_;

    // END_TUTORIAL
    };
} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H

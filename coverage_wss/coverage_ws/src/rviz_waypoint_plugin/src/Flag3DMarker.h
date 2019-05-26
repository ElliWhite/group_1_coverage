#ifndef MARKER_CONTROLLER_H_
#define MARKER_CONTROLLER_H_

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace visualization_msgs;
class Flag3DMarker{
public:
	std::string marker_name;
	InteractiveMarker int_marker;
	ros::Publisher goal_pub;
	Flag3DMarker();
	Flag3DMarker(std::string);
	Flag3DMarker(std::string, const Ogre::Vector3& position);
	Flag3DMarker(std::string,std::string);
	Flag3DMarker(std::string,std::string,std::string);
	~Flag3DMarker();

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler menu_handler;

	Marker makeBox(InteractiveMarker& msg);
	InteractiveMarkerControl& makeBoxControl(InteractiveMarker&);
	void makeViewFacingMarker(std::string mesh, const Ogre::Vector3& position);
	void makeViewFacingMarker();
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
	void reset();
};


#endif //MARKER_CONTROLLER

#include "Flag3DMarker.h"

static int counter=0;

Flag3DMarker::Flag3DMarker(){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>("/goal_topic", 1);
	std::stringstream ss;
	ss << "marker_controller" << counter++;
	server.reset( new interactive_markers::InteractiveMarkerServer(ss.str(),"",false) );
	menu_handler.insert("Select Goal",boost::bind(&Flag3DMarker::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

Flag3DMarker::Flag3DMarker(std::string goal_topic_name){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer("marker_controller","",false) );
	menu_handler.insert("Select Goal",boost::bind(&Flag3DMarker::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

Flag3DMarker::Flag3DMarker(std::string mesh, const Ogre::Vector3& position){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>("/goal_topic", 1);
	server.reset( new interactive_markers::InteractiveMarkerServer("marker_controller","",false) );
	menu_handler.insert("Select Goal",boost::bind(&Flag3DMarker::processFeedback,this,_1));
	makeViewFacingMarker(mesh,position);
	server->applyChanges();
}

Flag3DMarker::Flag3DMarker(std::string goal_topic_name, std::string int_server_name){
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer(int_server_name,"",false) );
	menu_handler.insert("Select Goal",boost::bind(&Flag3DMarker::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

Flag3DMarker::Flag3DMarker(std::string goal_topic_name, std::string int_server_name, std::string m_name){
	marker_name = m_name;
	ros::NodeHandle node;
	goal_pub = node.advertise<geometry_msgs::PoseStamped>(goal_topic_name, 1);
	server.reset( new interactive_markers::InteractiveMarkerServer(int_server_name,"",false) );
	menu_handler.insert("Select Goal",boost::bind(&Flag3DMarker::processFeedback,this,_1));
	makeViewFacingMarker();
	server->applyChanges();
}

Flag3DMarker::~Flag3DMarker(){}

void Flag3DMarker::reset()
{
	server.reset();
}


Marker Flag3DMarker::makeBox(InteractiveMarker& msg){
	Marker marker;
	marker.type = Marker::CUBE;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	return marker;

}

InteractiveMarkerControl& Flag3DMarker::makeBoxControl(InteractiveMarker& msg){
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back( makeBox(msg) );
	msg.controls.push_back( control );

	return msg.controls.back();
}

void Flag3DMarker::makeViewFacingMarker(){
	int_marker.header.frame_id = "/map";
	int_marker.header.stamp = ros::Time::now();

	int_marker.pose.position.x = 0;
	int_marker.pose.position.y = 0;
	int_marker.pose.position.z = 0;
	int_marker.scale = 1;

	std::stringstream ss;
	ss << marker_name << counter;
	int_marker.name = ss.str();
	int_marker.description = "Goal selection " + marker_name;

	InteractiveMarkerControl control;
	control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	control.independent_marker_orientation = true;
	control.name = int_marker.name+"-move";

	control.markers.push_back( makeBox(int_marker) );
	control.always_visible = true;

	int_marker.controls.push_back(control);

	control.interaction_mode = InteractiveMarkerControl::MENU;
	control.name = int_marker.name+"-menu";
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&Flag3DMarker::processFeedback,this,_1));
	menu_handler.apply(*server, int_marker.name);
}

void Flag3DMarker::makeViewFacingMarker(std::string mesh, const Ogre::Vector3& position){
	int_marker.header.frame_id = "/map";
	int_marker.header.stamp = ros::Time::now();

	int_marker.pose.position.x = position.x;
	int_marker.pose.position.y = position.y;
	int_marker.pose.position.z = position.z;
	int_marker.scale = 1;

	int_marker.name = marker_name;
	int_marker.description = "Goal selection " + marker_name;

	InteractiveMarkerControl control;
	control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	control.independent_marker_orientation = true;
	control.name = "move";

	Marker marker;
	marker.type = Marker::MESH_RESOURCE;
	marker.mesh_resource=mesh;
	marker.mesh_use_embedded_materials=true;
	marker.scale.x = int_marker.scale * 0.45;
	marker.scale.y = int_marker.scale * 0.45;
	marker.scale.z = int_marker.scale * 0.45;
	control.markers.push_back( marker );
	control.always_visible = true;

	int_marker.controls.push_back(control);

	control.interaction_mode = InteractiveMarkerControl::MENU;
	control.name = "menu";
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&Flag3DMarker::processFeedback,this,_1));
	menu_handler.apply(*server, int_marker.name);
}

void Flag3DMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback){
	std::ostringstream s;
	s << "Feedback from marker '" << feedback->marker_name << "' "
			<< " / control '" << feedback->control_name << "'";

	std::ostringstream mouse_point_ss;
	if( feedback->mouse_point_valid )
	{
		mouse_point_ss << " at " << feedback->mouse_point.x
				<< ", " << feedback->mouse_point.y
				<< ", " << feedback->mouse_point.z
				<< " in frame " << feedback->header.frame_id;
	}

	 switch ( feedback->event_type )
	  {
	    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
	      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
	      break;

	    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
	      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
	      if(feedback->menu_entry_id == 1)
	      {
	    	 geometry_msgs::PoseStamped goal;
	    	 goal.header.frame_id = "/map";
	    	 goal.header.stamp = feedback->header.stamp;
	    	 goal.pose = feedback->pose;
	    	 goal_pub.publish(goal);

	      }
	      break;
	    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
	      ROS_INFO_STREAM( s.str() << ": pose changed"
	          << "\nposition = "
	          << feedback->pose.position.x
	          << ", " << feedback->pose.position.y
	          << ", " << feedback->pose.position.z
	          << "\norientation = "
	          << feedback->pose.orientation.w
	          << ", " << feedback->pose.orientation.x
	          << ", " << feedback->pose.orientation.y
	          << ", " << feedback->pose.orientation.z
	          << "\nframe: " << feedback->header.frame_id
	          << " time: " << feedback->header.stamp.sec << "sec, "
	          << feedback->header.stamp.nsec << " nsec" );

	      break;

	    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
	      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
	      break;

	    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
	      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
	      break;
	  }

	  server->applyChanges();

}

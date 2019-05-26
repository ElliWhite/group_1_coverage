
#include <PointCloudFilter.h>


int main(int argc, char **argv){

	ros::init(argc, argv, "rgb_depth_kinect_fusion");

	PointCloudFilter node;

	ros::spin();


	return 0;


}
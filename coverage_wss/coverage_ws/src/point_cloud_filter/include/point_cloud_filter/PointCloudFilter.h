

class PointCloudFilter {

	private:
		ros::Nodehandle node;

		ros::Subscriber pcd_sub;

		ros::Publisher pcd_pub;

		image_transport::Subscriber image_sub;


		std::string image_topic;
		std::string pcd_colored_topic;
		std_string pcl_topic;



	public:
		PointCloudFilter();
		-PointCloudFilter();
};
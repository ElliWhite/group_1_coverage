

PointCloudFilter::PointCloudFilter(){

	image_topic = getParam<std::string>(node, "image_topic", "/rgb/image_rect_color");
	pcl_topic = getParam<std::string>(node, "pcl_topic", "/depth/pcd");

	image_transport::ImageTransport it(node);
	image_sub_ = it.subscribe(image_topic, 1, &PointCloudFilter::________________________________)

}

PointCloudFilter::-PointCloudFilter(){

}
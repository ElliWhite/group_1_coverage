/****
 Combine sensor_msgs/Image and sensor_msgs/PointCloud2
 ****/
/***
 Author: Li Jue Kun
 ****/

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
//#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl-1.7/pcl/point_cloud.h>
#include <vector>

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
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}



class PointCloudRGBCombiner
{
    
    typedef pcl::PointXYZRGB Point;
    
private:
	std::string image_topic;
	std::string pcd_color_topic;
	std::string pcl_topic;
	std::string pcd_rect_topic;
	double far_clip;
    ros::NodeHandle nh_;
    ros::Publisher color_pcl_pub_;
    ros::Subscriber image_sub_;
    ros::Publisher color_pcd_pub_;
    ros::Publisher rect_pcd_pub_;
    
    //    pcl::PointCloud<pcl::PointXYZRGB> *colored_pcl_in;
    
public:
    
    PointCloudRGBCombiner(ros::NodeHandle nh): nh_(nh) // ,colored_pcl_in(0L)
    {
        // std::string image_topic = nh_.resolveName(“img_in”);
        // std::string image_topic = "/rgb/image_rect_color";
        image_topic = getParam<std::string>(nh_, "image_topic", "/vrep/turtlebot2/rgb/image_rect_color"); 
        pcl_topic = getParam<std::string>(nh_, "pcl_topic", "/vrep/turtlebot2/depth/pcd");
        pcd_color_topic = getParam<std::string>(nh_, "pcd_color_topic", "/vrep/turtlebot2/depth/pcd_colored");
        pcd_rect_topic = getParam<std::string>(nh_, "pcd_rect_topic", "/vrep/turtlebot2/depth/pcd_rect");
        far_clip = getParam<double>(nh_, "far_clip", 3.5);
        color_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcd_color_topic, 10);
        rect_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pcd_rect_topic, 10);
        image_sub_ = nh_.subscribe(image_topic, 1, &PointCloudRGBCombiner::imageCb, this);
        //        colored_pcl_in = new pcl::PointCloud<pcl::PointXYZRGB>();
        //        color_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vrep/ugv0/arm_camera/depth/new_pcd_colored", 10);
    }
    
    ~PointCloudRGBCombiner() {}
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
        
        //cv_bridge::CvImagePtr cv_color;
        //std::vector<uint8_t> color_vect;
        // ros::Time start_time = ros::Time::now();
        // std::string pcl_topic = nh_.resolveName(“pcl_in”);
        // std::string pcl_topic = "/depth/pcd";
        
        //ROS_INFO("Init: Waiting for point cloud2 and image");        
        
        // while (ros::ok()){
            
            sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(5.0));
            
            while (!recent_cloud){
                ROS_INFO("Wait: Waiting for point cloud2 and image");
                sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(10.0));
            }
            
 	    //ROS_INFO("End: Wait: Waiting for point cloud2 and image");
            // ROS_INFO_STREAM("pcl received after " << ros::Time::now() – start_time << " seconds. Start Combining...");

	    //std::cout << "Start merging " << std::endl;
            merge(*const_cast<sensor_msgs::PointCloud2*>(recent_cloud.get()), msg->data);
	    // merge(*cloud, msg->data);
            
            //            std::cout << "PCL size: " << colored_pcl_in->points.size() << std::endl;
            
            //            sensor_msgs::PointCloud2 colored_pcd_out;
            //            pcl::toROSMsg(*colored_pcl_in, colored_pcd_out);
            //            colored_pcd_out.header = pcl_conversions::fromPCL(colored_pcl_in->header);
            //            color_pcd_pub_.publish(colored_pcd_out);
            // convertPoints(*const_cast<sensor_msgs::PointCloud2*>(recent_cloud.get()));
            //std::cout << "end merging " << std::endl;

	    //std::cout << "Start converting " << std::endl;
	    convertPoints(*const_cast<sensor_msgs::PointCloud2*>(recent_cloud.get()));
	    //std::cout << "End converting " << std::endl;
	    
/*
	    pcl::PointCloud<pcl::PointXYZRGB> *pcl_in = new pcl::PointCloud<pcl::PointXYZRGB>();
	    pcl_in->header = pcl_conversions::toPCL(recent_cloud->header);
	    pcl::fromROSMsg(*recent_cloud, *pcl_in);

	    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	    pcl::PointCloud<pcl::PointXYZRGB> *pcl_temp = new pcl::PointCloud<pcl::PointXYZRGB>();
            for (it = pcl_in->points.begin(); it < pcl_in->points.end(); it++)
	    {
		pcl::PointXYZRGB p;
		p = *it;
		
		if (p.z < 3.5 * 0.9)
	        {
			pcl_temp->points.push_back(p);
		}
		// std::cout << "p.z " << p.z << std::endl;
	    }

	    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
	    pcl::toROSMsg(*pcl_temp, *cloud);

	    cloud->header.stamp = recent_cloud->header.stamp;
	    cloud->header.frame_id = recent_cloud->header.frame_id;
	    std::cout << "recent_cloud->header.frame_id " << recent_cloud->header.frame_id <<std::endl;
	    std::cout << "cloud->header.frame_id " << cloud->header.frame_id <<std::endl;            
*/	    
            // color_pcl_pub_.publish(*cloud);
            //cv_color.reset();
            recent_cloud.reset();
            //color_vect.clear();
        //}
        
    }
    
    void merge(sensor_msgs::PointCloud2 &cloud, const std::vector<uint8_t> &colors){
        
        //        if(colored_pcl_in) delete colored_pcl_in;
        //        colored_pcl_in = new pcl::PointCloud<pcl::PointXYZRGB>();
        //        colored_pcl_in->header = pcl_conversions::toPCL(cloud.header);
        
        size_t size = size_t(colors.size()/3);
        size_t col = size_t(640);
        size_t row = size_t(480);
        //        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, std::string("x"));
        //        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, std::string("y"));
        //        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, std::string("z"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, std::string("r"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, std::string("g"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, std::string("b"));
        int count =0 ;
        for (size_t j = 0; j< row; ++j ){
            for (size_t i =0; i < col; ++i, ++iter_r, ++iter_g, ++iter_b){
                count++;
                *iter_r = colors[3*(i+(479-j)*640)+0];
                *iter_g = colors[3*(i+(479-j)*640)+1];
                *iter_b = colors[3*(i+(479-j)*640)+2];
                // std::cout << "x: " << *iter_x << "y: " << *iter_y << "z: " << *iter_z << std::endl;
                //                pcl::PointXYZRGB p = pcl::PointXYZRGB(*iter_r, *iter_g, *iter_b);
                //                p.x = *iter_x;
                //                p.y = *iter_y;
                //                p.z = *iter_z;
                //                colored_pcl_in->points.push_back(p);
                //point cloud count from left to right, bottom to up while color image count from left to right, up to bottom
                //above conversion to force counting consistency of color with point cloud
            }
        }
    }
    
    void convertPoints(sensor_msgs::PointCloud2 &inMsg) {
        
        // allocate an PointXYZRGB message with same time and frame ID as
        // input data
        sensor_msgs::PointCloud2::Ptr outMsg(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2Modifier modifier(*outMsg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(inMsg.height * inMsg.width);
        
        outMsg->header.stamp = inMsg.header.stamp;
        outMsg->header.frame_id = inMsg.header.frame_id;
        outMsg->height = 1;
        
        sensor_msgs::PointCloud2Iterator<float> out_x(*outMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*outMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*outMsg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*outMsg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*outMsg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*outMsg, "b");
        
        sensor_msgs::PointCloud2ConstIterator<float> in_x(inMsg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(inMsg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(inMsg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> in_r(inMsg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> in_g(inMsg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> in_b(inMsg, "b");
        
        for (size_t i = 0; i < inMsg.height * inMsg.width; ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b,
                ++in_x, ++in_y, ++in_z, ++in_r, ++in_g, ++in_b)
        {
            *out_x = *in_x;
            *out_y = *in_y;
            *out_z = *in_z;
            
            *out_r = *in_r;
            *out_g = *in_g;
            *out_b = *in_b;
        }
        
        // color_pcl_pub_.publish(*outMsg);
        pcl::PointCloud<pcl::PointXYZRGB> *pcl_in = new pcl::PointCloud<pcl::PointXYZRGB>();
	pcl_in->header = pcl_conversions::toPCL(inMsg.header);
	pcl::fromROSMsg(*outMsg, *pcl_in);
	
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	pcl::PointCloud<pcl::PointXYZRGB> *pcl_temp = new pcl::PointCloud<pcl::PointXYZRGB>();
	pcl::PointCloud<pcl::PointXYZ> *pcl_temp_rect = new pcl::PointCloud<pcl::PointXYZ>();
        for (it = pcl_in->points.begin(); it < pcl_in->points.end(); it++)
	{
		pcl::PointXYZRGB p;
		pcl::PointXYZ q;
		p = *it;
		q.x = it->x;
		q.y = it->y;
		q.z = it->z;
		
		if (p.z < 0.99  * far_clip)
	    {
			pcl_temp->points.push_back(p);
			pcl_temp_rect->points.push_back(q);
		}
		// std::cout << "p.z " << p.z << std::endl;
	}

	sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2());
	pcl::toROSMsg(*pcl_temp, *cloud);
	
	sensor_msgs::PointCloud2::Ptr cloud_rect(new sensor_msgs::PointCloud2());
	pcl::toROSMsg(*pcl_temp_rect, *cloud_rect);

	cloud->header.stamp = inMsg.header.stamp;
	cloud->header.frame_id = inMsg.header.frame_id;
	
	cloud_rect->header.stamp = inMsg.header.stamp;
	cloud_rect->header.frame_id = inMsg.header.frame_id;
	// std::cout << "recent_cloud->header.frame_id " << inMsg.header.frame_id;
	// std::cout << "cloud->header.frame_id " << cloud->header.frame_id <<std::endl;            
	    
        color_pcl_pub_.publish(*cloud);
        rect_pcd_pub_.publish(*cloud_rect);
        
        pcl_in->points.clear();
		pcl_temp->points.clear();
		pcl_temp_rect->points.clear();
		cloud.reset();
		cloud_rect.reset();
        delete pcl_in;
        delete pcl_temp;
        delete pcl_temp_rect;
    }
    
    
    
};


int main(int argc, char **argv){
    ros::init(argc, argv, "rgb_pcd_kinect_fusion_t1");
    ros::NodeHandle nh;
    
    PointCloudRGBCombiner node(nh);
    ros::spin();
    return 0;
}


#ifndef NEUVITION_DRIVER_H
#define NEUVITION_DRIVER_H
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "neuv_defs.hpp"



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


namespace neuvition_driver {
    class neuvitionDriver
    {

    public:
        neuvitionDriver(rclcpp::Node::SharedPtr node);
        void neuInit() ;
        void neuConnect() ;
        void neuStartScan();
        void neuStartData();
        void neuStopScan();
        void neuStopData();
        void neuDisconnect();
        void neuSetLaserPeriod(int value);
  	void neuSetPwm(int value);
        void neuSetDataFrame(int value);
        void neuVideoFusion(bool value);
        void neuProcessPoint(PointCloudT &cloud) ;
        void neuProcessImage(sensor_msgs::msg::Image::SharedPtr msg_) ;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_;
        sensor_msgs::msg::PointCloud2 neuvition_pub;
	 int upstream_port;
        std::string upstream_host;
        rclcpp::Node::SharedPtr  pnode;
        image_transport::Publisher output_img_;
    };

}

#endif

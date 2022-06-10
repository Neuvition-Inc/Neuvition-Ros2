#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sstream> 




#include <math.h>

#include <string>
#include <iostream>

#include <boost/version.hpp>
#include <boost/timer/timer.hpp>

#include "driver.h"
//#include <tf/transform_listener.h>
#include <pcl/point_types_conversion.h>    //pcl_conversions.h
//#include <sensor_msgs/Imu.h>

#include <stdlib.h>
#include <stdint.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


using namespace std;
using namespace pcl;



#define UNUSED(var) (void)(var)

int isImageRotate = 0;

int isTimemode = 1;

uint32_t iFrontFrameTime = 0;
uint32_t iNowFrameTime = 0;
uint32_t iMsecTime = 0 ;

/* YELLOW ff/ff/00 RED ff/00/00 MAGENTA ff/00/ff BLUE 00/00/ff CYAN 00/ff/ff GREEN 00/ff/00 */
const uint32_t coloredge[6]={ 0xffff00, 0xff0000, 0xff00ff, 0x0000ff, 0x00ffff, 0x00ff00 };

uint32_t tof_cycle=65000;  // orig:9000 2m:13000 10m:65000 30m:200000

double get_timestamp(void) 
{
    struct timeval now;
    gettimeofday(&now, 0);
    
    return (double)(now.tv_sec) + (double)(now.tv_usec)/1000000.0;
}

void showretval(int ret)
{ 
    if (ret==0)
        return;
    std::cout<< "ret:"<< ret << std::endl;
}

namespace neuvition_driver 
{
	class gEventHandler : public neuvition::INeuvEvent
	{
	
		private:
        neuvitionDriver *neudrv;
         public:
        gEventHandler(neuvitionDriver* _drv):neudrv(_drv)
        {
            std::cout << "The callback instance of driver initialized" << std::endl;
        }
		 virtual void on_connect(int code, const char* msg) 
        	{
            	std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvtion: on_connect: " << code << "(" << msg << ")" << std::endl;
            		if (code==0)
            		{
                		int ret = neuvition::set_reconnect_params(false, 5); showretval(ret);
            		}
        	}
        	 virtual void on_disconnect(int code) 
        	{
            		std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: on_disconnect: "<< code << std::endl;
       	}
       	virtual void on_lidar_info_status(neuvition::LidarInfoStatus*)
		{
			//TODO: nothing		
		}
		  virtual void on_response(int code, enum neuvition::neuv_cmd_code cmd) 
        {
            std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: on_response[ " << cmd << " ]: " << code << std::endl;
            if (cmd==neuvition::NEUV_CMD_START_SCAN && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: scanning started..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_STOP_SCAN && code==0 )     { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: scanning stopped..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_START_STREAM && code==0 )  { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: streaming started..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_STOP_STREAM && code==0 )   { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: streaming stopped..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_GET_PARAMS && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: device parameter is  synced..." << std::endl; }
            if (cmd==neuvition::NEUV_CMD_GPS_UPDATED && code==0 )    { std::cout << std::fixed << std::setprecision(6) << get_timestamp() << "neuvition: gps info message..." << std::endl; }
        }
         virtual void on_framedata(int code,int64_t microsec, const neuvition::NeuvUnits& data, const neuvition::nvid_t& frame_id) 
        {
        	
        	std::cout << "code = " << code << " " << "microsec = " << microsec << std::endl;
        	printf("%d\n",frame_id);
        	if (data.size() == 0) 
           	 return;
           	 
           	PointCloudT cloud_;
            	cloud_.reserve(data.size());
           	 
           for (neuvition::NeuvUnits::const_iterator iter = data.begin(); iter != data.end(); iter++) 
            {
                const neuvition::NeuvUnit& np = (*iter);
                PointT point;

                //PointXYZRGBATI
                point.x = np.x*0.001; point.y = np.y*0.001; point.z = np.z*0.001;

                if(np.z==0) 
                    continue;

                point.a = 255;
   			uint32_t tofcolor=np.tof%tof_cycle;
                    uint32_t tofclass=tofcolor/(tof_cycle/6);
                    uint32_t tofreman=tofcolor%(tof_cycle/6);
                    uint32_t cbase=coloredge[(tofclass+0)%6];
                    uint32_t cnext=coloredge[(tofclass+1)%6];
                    uint32_t ccand=cnext&cbase; UNUSED(ccand);
                    uint32_t ccxor=cnext^cbase;
                    int shift=__builtin_ffs(ccxor)-1;
                    int xincr=(cnext>cbase) ? 1 : -1;
                    uint32_t crender=cbase+xincr*((int)(tofreman*(256.0/(tof_cycle/6)))<<shift);
                    point.r = (crender&0xff0000)>>16;
                    point.g = (crender&0x00ff00)>>8;
                    point.b = (crender&0x0000ff)>>0;
                
               

                cloud_.push_back(point);  

            }

        
           
         

        

           neudrv->neuProcessPoint(cloud_);
           	 
           	 
        }
         virtual void on_pczdata(bool status) 
        {
		std::cout << status << std::endl;
        }
    	virtual void on_framestart1(int nCode) { std::cout << nCode << std::endl;}
	virtual void on_framestart2(int nCode) {std::cout << nCode << std::endl;}
       virtual void on_Ladar_Camera(  neuvition::NeuvCameraLadarDatas * neuvcameraladarpos)
       {  
       	if (neuvcameraladarpos->size() == 0) 
           	 return;
       }      
	virtual void on_mjpgdata(int code, int64_t microsec, cv::Mat Mat) 
        { 	
        	std::cout << "code = " << code << " " << "on_mjpgdata microsec = " << microsec << std::endl;
        	if (Mat.empty()) 
           	 return;
            if(true)
        {
            transpose(Mat,Mat);
            flip(Mat,Mat,1);
            transpose(Mat,Mat);
            flip(Mat,Mat,1);
        }

            //  sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Mat).toImageMsg();
             sensor_msgs::msg::Image::SharedPtr msg_ =  cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", Mat).toImageMsg();

              neudrv->neuProcessImage(msg_);
        }
        virtual void on_imudata(int code,int64_t microsec,const neuvition::NeuvUnits& data,const neuvition::ImuData& imu) 
        { 	
        	std::cout << "code = " << code << " " << "microsec = " << microsec << std::endl;
        	std::cout <<  imu.quat_i << std::endl;
        	if (data.size() == 0) 
           	 return;
        }
	};
neuvitionDriver::neuvitionDriver(rclcpp::Node::SharedPtr node) 
    {
    	pnode = node;
    	RCLCPP_INFO(node->get_logger()," ros foxy version  1.0.1");
   
   
    	output_cloud_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("neuvition_cloud", 20);
        image_transport::ImageTransport it_(node);
        //image_transport::create_publisher(node, "neuvition_image",1);
        output_img_ = it_.advertise("neuvition_image", 1);
    }
    
void neuvitionDriver::neuConnect() 
{
    printf ("Start connecting ...\n");
    neuvition::set_camera_status(true);
    neuvition::set_flip_axis(false, true);
    neuvition::set_mjpg_curl(true);

    neuvition::INeuvEvent * phandler = new gEventHandler(this);

      int ret=neuvition::setup_client("192.168.1.101", 6668, phandler, false);

    UNUSED(ret);
    usleep(2000000);
}
void neuvitionDriver::neuStartScan() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..start scan"<<std::endl;
    int ret=neuvition::start_scan(); showretval(ret); // ask device to start scanning
    usleep(200000);
}

void neuvitionDriver::neuStartData() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..start stream"<<std::endl;
    int ret=neuvition::start_stream(); showretval(ret); // ask device to start streaming
    usleep(200000);

}
void neuvitionDriver::neuStopScan() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..stop scan"<<std::endl;
    int ret=neuvition::stop_scan(); showretval(ret); // ask device to stop scanning
    usleep(200000);
}


void neuvitionDriver::neuStopData() {
    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..stop stream"<<std::endl;
    int ret=neuvition::stop_stream(); showretval(ret); // ask device to stop streaming
    usleep(200000);

}
void neuvitionDriver::neuDisconnect() {

    std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..teardown client"<<std::endl;
    int ret=neuvition::teardown_client(); showretval(ret);
    sleep(1);

}

void neuvitionDriver::neuSetLaserPeriod(int value) {

    //0: 200khz   1: 300KHZ 2 500KHZ 4 1MHZ
   
   
    int ret=neuvition::set_laser_interval(value);//3
    UNUSED(ret);
    usleep(2000000);
}


void neuvitionDriver::neuSetDataFrame(int value) {

   
    int ret = neuvition::set_frame_frequency(value);
    UNUSED(ret);
    usleep(2000000);
    std::cout << "frame = "<< neuvition::get_frame_frequency() <<endl;

}


void neuvitionDriver::neuSetPwm(int value) {
    //<=65%
    printf("PWM Value: %d \n", value);
    int ret = neuvition::set_laser_power(value);
    UNUSED(ret);
    usleep(2000000);

}
void neuvitionDriver::neuInit() 
{

    neuConnect();

    usleep(200000);

    if(neuvition::is_connected()) 
    {
        std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..connected"<<std::endl;

        neuSetPwm(50);
        usleep(200000);
       
        neuSetLaserPeriod(2);
        usleep(200000);
        neuSetDataFrame(10);
  	usleep(200000);
       


        neuStartScan();
        if(neuvition::is_scanning())  std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..scanning" <<std::endl;

      

        neuStartData();
        usleep(200000);

     
        if(neuvition::is_streaming())
        { 
            std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" ..streaming"<<std::endl;
        }

        sleep(3);

      	neuVideoFusion(true);

    } 
    else 
    {
        std::cout<<std::fixed<<std::setprecision(6)<<get_timestamp()<<" failed to connect .. "<<std::endl;
        exit(1);
    }

}
void neuvitionDriver::neuProcessPoint(PointCloudT &cloud) 
{
      //need convert to pcl::PointXYZI, 
    //otherwise the ROS_subscriber will popup warning 'Failed to find match for field intensity'
    pcl::PointCloud<PointT>   pcl_points;
    pcl_points.resize(cloud.size());
    int iSize = (int)cloud.size();

    for (int i = 0; i < iSize; i++) 
    {
        pcl_points.at(i).x = cloud.at(i).x;
        pcl_points.at(i).y = cloud.at(i).y;
        pcl_points.at(i).z = cloud.at(i).z;
	pcl_points.at(i).r = cloud.at(i).r;
	pcl_points.at(i).g = cloud.at(i).g;
	pcl_points.at(i).b = cloud.at(i).b;
	pcl_points.at(i).a = 255;
	
    }
    
    pcl::toROSMsg(pcl_points, neuvition_pub); 

    neuvition_pub.header.stamp = pnode->now();
    neuvition_pub.header.frame_id = "neuvition";
  

    output_cloud_->publish(neuvition_pub);
}

void neuvitionDriver::neuProcessImage(sensor_msgs::msg::Image::SharedPtr msg_) {

     msg_->header.stamp = pnode->now();
     output_img_.publish(msg_);


}

void neuvitionDriver::neuVideoFusion(bool value) {

    printf("Camera: [%s]\n",true?"Open":"Close");
    int ret  = neuvition::set_camera_status(value);
    ret=neuvition::set_mjpg_curl(value); 
    UNUSED(ret);
    usleep(200000);
}

}
 





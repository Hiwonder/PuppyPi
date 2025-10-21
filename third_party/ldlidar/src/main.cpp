#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tofbf.h"
#include <string>

int main(int argc , char **argv)
{
	ros::init(argc, argv, "product");
	ros::NodeHandle nh;                    /* create a ROS Node */

 	
	LiPkg * lidar = new LiPkg;
  
    CmdInterfaceLinux cmd_port;
    std::vector<std::pair<std::string, std::string> > device_list;
    std::string port_name;
    cmd_port.GetCmdDevices(device_list);
    for (auto n : device_list)
    {
        std::cout << n.first << "    " << n.second << std::endl;
        if( (strstr(n.second.c_str(),"CP2102")) || (strstr(n.second.c_str(),"USB2.0-Serial")) )
        {
            port_name = n.first;
        }
    }
	//port_name = "/dev/ttyAMA0";
	port_name = "/dev/ttyUSB0";
	if(port_name.empty())
	{
		std::cout<<"Can't find LiDAR LD06"<< std::endl;
	}

	std::cout<<"FOUND LiDAR_LD06"  <<std::endl;
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		if(lidar->Parse((uint8_t*)byte, len))
		{
			lidar->AssemblePacket();  
		}
	});

	if(cmd_port.Open(port_name))
		std::cout<<"LiDAR_LD06 started successfully "  <<std::endl;
	
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 5); /*create a ROS topic */
	// ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
	
	ros::Rate loop_rate(50);
	
	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
			lidar->ResetFrameReady();
		}
		loop_rate.sleep();
	}
    return 0;
}


#ifndef RTK_PROCESS_H
#define RTK_PROCESS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>

#include <utility.h>

#include <sensor_msgs/NavSatFix.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <iomanip>

class RTK_Process
{
    public:

    ros::NodeHandle nh;

    RTK_Process();

    char RTK_serial_init(std::string port);

    char Get_RTK_Data(ros::Publisher& GGA_puber, ros::Publisher& VTG_puber);

    void RTK_GGA_data_process(std::string s, GNSS_s & t_RTK);

    void RTK_VTG_data_process(std::string s, float &velocity);

    void publishRTK(ros::Publisher& puber, GNSS_s t_RTK);

    void publishVTG(ros::Publisher& puber, float velocity);

    std::string GGA_start = "GGA", VTG_start = "VTG";
    std::string RTK_end = "*";

    ros::Publisher RTK_pub;
    ros::Publisher VTG_pub;

    std::string serial_port_RTK = "/dev/toRTK";
    serial::Serial RTK_serial;
    char GGA_flag = 0, VTG_flag = 0;
    char RTK_GGA_Success_flag = 0, RTK_VTG_Success_flag = 0;
    GNSS_s RTK_new;
    float velocity = 0.0;
    std::string strReceive;
    int RTK_count = 0;
};

#endif

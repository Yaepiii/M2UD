#include "RTK_process.h"

RTK_Process::RTK_Process()
{
    RTK_pub = nh.advertise<sensor_msgs::NavSatFix>("/RTK/data_raw", 1000);
    VTG_pub = nh.advertise<std_msgs::Float32>("/RTK/velocity", 1000);
    RTK_serial_init(serial_port_RTK);
}

char RTK_Process::RTK_serial_init(std::string port)
{
    try
    {
        RTK_serial.setPort(port);
        RTK_serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        RTK_serial.setTimeout(to);
        RTK_serial.open();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("RTK %s initialized failed !", port.c_str());
        return -1;
    }
    if (RTK_serial.isOpen())
    {
        ROS_INFO("RTK %s initialized successful !", port.c_str());
        ROS_INFO("RTK %s : 115200", port.c_str());
        return 1;
    }
    else
    {
        ROS_ERROR("RTK %s can not open ! ", port.c_str());
        return -1;
    }
}

char RTK_Process::Get_RTK_Data(ros::Publisher& GGA_puber, ros::Publisher& VTG_puber)
{
    int i = 0, start = -1, end = -1;

    if (RTK_serial.available())
    {
        strReceive += RTK_serial.read(RTK_serial.available());
        while (strReceive.length() > 3)
        {
            start = strReceive.find(GGA_start);
            if (start != std::string::npos)
            {
                GGA_flag = 1;
                // printf("--find start: GGA_flag = 1 ! \n");
            }
            else
            {
                start = strReceive.find(VTG_start);
                if (start != std::string::npos)
                {
                    VTG_flag = 1;
                }
                else
                {
                    // printf("--not find start flag... \n");
                    // printf("--temp strReceive: %s \n\n",strReceive.c_str());
                    strReceive.clear();
                    break;
                }
            }

            if (GGA_flag == 1 || VTG_flag == 1)
            {
                strReceive = strReceive.substr(start);
                end = strReceive.find(RTK_end);
                if (end == std::string::npos)
                {
                    // printf("--not find end flag... \n");
                    // printf("--temp strReceive: %s \n\n", strReceive.c_str());
                    break;
                }
                else
                {
                    // printf("--find end flag ! \n");
                    if (GGA_flag == 1)
                    {
                        RTK_GGA_data_process(strReceive.substr(start, end-start), RTK_new);
                        RTK_GGA_Success_flag = 1;
                        GGA_flag = 0;
                        printf("--GGA_%s | count:%d, lon:%f, lat:%f, alt:%f, HDOP:%f, num:%d \n", serial_port_RTK.c_str(), RTK_count++,
                               RTK_new.longitude, RTK_new.latitude, RTK_new.altitude, RTK_new.HDOP, RTK_new.Num_satellites);
                    }
                    else if (VTG_flag == 1)
                    {
                        RTK_VTG_data_process(strReceive.substr(start, end-start), velocity);
                        RTK_VTG_Success_flag = 1;
                        VTG_flag = 0;
                    }

                    if (RTK_GGA_Success_flag == 1)
                    {
                        publishRTK(GGA_puber, RTK_new);
                        RTK_GGA_Success_flag = 0;
                    }
                    else if (RTK_VTG_Success_flag == 1)
                    {
                        publishVTG(VTG_puber, velocity);
                        RTK_VTG_Success_flag = 0;
                    }

                    strReceive = strReceive.substr(end+1);
                    // printf("--finish strReceive: %s \n\n", strReceive.c_str());
                }
            }
        }
    }
    return 0;
}

void RTK_Process::RTK_GGA_data_process(std::string s, GNSS_s & t_RTK)
{
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    double t, lon, lat, alt, num, hdop, status;
    pos2 = s.find(",");
    pos1 = 0;
    while (pos2 != std::string::npos)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
        pos1 = pos2 + 1;
        pos2 = s.find(",", pos1);
    }
    if (pos1 != s.length())
    {
        v.push_back(s.substr(pos1));
    }

    // if (v.max_size() >= 10)
    if (v.size() >= 10)
    {
        if (v[1] != "")
        {
            t = std::atof(v[1].c_str()) / 10000;
            int h = (int)floor(t);
            int m = (int)floor((t - h) * 100);
            double s = ((t - h) * 100 - m) * 100;
            t = h * 3600 + m * 60 + s;
        }
        if (v[2] != "") lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;
        lat = ilat + (lat - ilat) * 100.0 / 60.0;
        if (v[4] != "") lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100.0 / 60.0;
        if (v[6] != "") status = std::atoi(v[6].c_str());
        if (v[7] != "") num = std::atoi(v[7].c_str());
        if (v[8] != "") hdop = std::atof(v[8].c_str());
        if (v[9] != "") alt = std::atof(v[9].c_str());

        t_RTK.t = t;
        t_RTK.latitude = lat;
        t_RTK.longitude = lon;
        t_RTK.altitude = alt;
        t_RTK.Num_satellites = num;
        t_RTK.HDOP = hdop;
        t_RTK.status = status;
    }
}

void RTK_Process::RTK_VTG_data_process(std::string s, float &velocity)
{
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while (pos2 != std::string::npos)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
        pos1 = pos2 + 1;
        pos2 = s.find(",", pos1);
    }
    if (pos1 != s.length())
    {
        v.push_back(s.substr(pos1));
    }

    if (v.size() >= 7)
    {
        if (v[5] != "") velocity = std::atof(v[5].c_str()) * 0.5144444; // m/s
    }
}

void RTK_Process::publishRTK(ros::Publisher& puber, GNSS_s t_RTK)
{
    sensor_msgs::NavSatFix RTK_send;

    double covariance[9] = {(float)t_RTK.Num_satellites, t_RTK.HDOP, t_RTK.VDOP, t_RTK.yaw, t_RTK.pitch, t_RTK.t, 0, 0, 0};

    RTK_send.header.stamp = ros::Time::now();
    RTK_send.header.frame_id = "global";

    RTK_send.longitude = t_RTK.longitude;
    RTK_send.latitude = t_RTK.latitude;
    RTK_send.altitude = t_RTK.altitude;
    RTK_send.status.status = t_RTK.status;

    for (int i = 0; i < 9; i++)
    {
        RTK_send.position_covariance[i] = covariance[i];
    }
    puber.publish(RTK_send);
}

void RTK_Process::publishVTG(ros::Publisher& puber, float velocity)
{
    std_msgs::Float32 VTG_send;
    VTG_send.data = velocity;
    puber.publish(VTG_send);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RTK_node");

    RTK_Process rtk_processer;

    std::cout << "--RTK_node node begain, waiting for serial data !!!" << std::endl;

    ros::Rate rate(1000);

    while (ros::ok())
    {
        rtk_processer.Get_RTK_Data(rtk_processer.RTK_pub, rtk_processer.VTG_pub);

        ros::spinOnce();

        rate.sleep();
    }

    ros::spin();

    std::cout << "-- RTK_node node end !!!" << std::endl;

    return 0;
}

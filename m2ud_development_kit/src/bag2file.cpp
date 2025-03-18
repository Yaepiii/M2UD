/****************************************************************************************
 *
 * Copyright (c) 2024, Shenyang Institute of Automation, Chinese Academy of Sciences
 *
 * Authors: Yanpeng Jia
 * Contact: yaepiii@126.com
 * 
 * Usage: 
 *  1) Get pose ground truth from RTK topic.
 *  2) Fix RTK data (using interpolation) at RTK raw frequency (5Hz) as ground truth.
 *  3) Update RTK data at LiDAR scan frequency (10Hz).
 *  4) Show ground truth trajectory on Rviz and save ground truth file as 'tum' format in .txt.
 *
 ****************************************************************************************/
#include "utility.h"

class Bag2File{
    public:
        ros::NodeHandle nh;
        
        std::string save_root_path;
        std::string rosbag_path;
        std::string rosbag_file;
        std::string RTK_topic;
        std::string IMU_topic;
        std::string Mag_topic;
        std::string LiDAR_topic;
        std::string Camera_topic;
        std::string Depth_topic;
        rosbag::Bag bag;
        
        std::vector<double> timestamps;
        int platform = 1;   // 1: 4 wheel robot 2: 6 wheel robot
        bool RTK_exist = 0;
        bool color_exist = 0;
        bool depth_exist = 0;
        bool pc_exist = 0;
        bool imu_exist = 0;
        bool mag_exist = 0;

        Bag2File(){
            ros::param::param<std::string>("rosbag_path", rosbag_path, "The root directory where you bag file");
            ros::param::param<std::string>("rosbag_file", rosbag_file, "The bag file");
            ros::param::param<std::string>("IMU_topic", IMU_topic, "The IMU topic");
            ros::param::param<std::string>("RTK_topic", RTK_topic, "The RTK topic");
            ros::param::param<std::string>("Mag_topic", Mag_topic, "The Mag topic");
            ros::param::param<std::string>("LiDAR_topic", LiDAR_topic, "The LiDAR topic");
            ros::param::param<std::string>("Camera_topic", Camera_topic, "The Camera RGB topic");
            ros::param::param<std::string>("Depth_topic", Depth_topic, "The Camera depth topic");
            std::string::size_type pos = rosbag_file.find(".");
            std::string bag_file_name = rosbag_file.substr(0, pos);
            save_root_path = rosbag_path;
            save_root_path = save_root_path + bag_file_name + "/";
            // Make file directory
            int32_t mkdir_status_root = openDirAndMkdir(save_root_path.c_str());
            int32_t mkdir_status_image = openDirAndMkdir((save_root_path+"image/").c_str());
            int32_t mkdir_status_image_color = openDirAndMkdir((save_root_path+"image/color/").c_str());
            int32_t mkdir_status_image_depth = openDirAndMkdir((save_root_path+"image/depth/").c_str());
            int32_t mkdir_status_pc = openDirAndMkdir((save_root_path+"pointcloud/").c_str());
            int32_t mkdir_status_imu = openDirAndMkdir((save_root_path+"imu/").c_str());
            int32_t mkdir_status_RTK = openDirAndMkdir((save_root_path+"RTK/").c_str());
            int32_t mkdir_status_calib = openDirAndMkdir((save_root_path+"calibration/").c_str());
            if (mkdir_status_root == -1 || mkdir_status_image == -1 || mkdir_status_image_color == -1 || mkdir_status_image_depth == -1 ||
                mkdir_status_pc == -1 || mkdir_status_imu == -1 || mkdir_status_RTK == -1 || mkdir_status_calib == -1){
                std::cerr << "Mkdir failed ......" << std::endl << std::endl;
                ROS_BREAK();
            }
            // open rosbag file
            bag.open(rosbag_path+rosbag_file, rosbag::bagmode::Read);
            if (bag.isOpen()){
                std::cerr << "Bag file opens successfully......" << std::endl << std::endl;
            }
            else{
                std::cerr << "Bag file opens failed......" << std::endl << std::endl;
            }
        }
        ~Bag2File(){
            bag.close();
        }

        // transform the lla to ecef
        Eigen::Vector3d lla2ecef(Eigen::Vector3d data){
            Eigen::Vector3d ecef;     // the ecef for output
            ecef.resize(3, 1);
            double a = 6378137.0;     // major axis of earth
            double b = 6356752.314;   // minor axis of earth
            double n, Rx, Ry, Rz;
            double lon = data.x() * M_PI / 180.0; // lon to radis
            double lat = data.y() * M_PI / 180.0; // lat to radis
            double alt = data.z(); // altitude
            n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
            Rx = (n + alt) * cos(lat) * cos(lon);
            Ry = (n + alt) * cos(lat) * sin(lon);
            Rz = (b * b / (a * a) * n + alt) * sin(lat);
            ecef.x() = Rx; // return value in ecef
            ecef.y() = Ry; // return value in ecef
            ecef.z() = Rz; // return value in ecef
            return ecef;
        }

        void save_pointcloud(){
            std::ofstream f_save_timestamp;
            f_save_timestamp.open(save_root_path+"timestamp.txt", std::fstream::out);
            if (f_save_timestamp.is_open()){
                std::cerr << "Timestamp file opens successfully......" << std::endl << std::endl;
            }
            else{
                std::cerr << "Timestamp file opens failed......" << std::endl << std::endl;
            }
            rosbag::View pc_view(bag, rosbag::TopicQuery(LiDAR_topic));
            if (pc_view.size() == 0){
                ROS_WARN("Not found point cloud data! Skip it!");
                return;
            }
            pc_exist = 1;
            for (rosbag::View::iterator it = pc_view.begin(); it != pc_view.end(); ++it){
                sensor_msgs::PointCloud2ConstPtr pc_msg = it->instantiate<sensor_msgs::PointCloud2>();
                if (pc_msg != NULL){
                    timestamps.push_back(pc_msg->header.stamp.toSec());
                    f_save_timestamp << std::fixed << std::setprecision(6) << pc_msg->header.stamp.toSec() << std::endl;
                    
                    std::string bin_path = save_root_path + "pointcloud/" + to_string(pc_msg->header.stamp.toSec()) + ".bin";
                    std::ofstream bin_file(bin_path.c_str(), ios::out | ios::binary | ios::app);
                    if(!bin_file.good()){
                        std::cout << "Couldn't open " << bin_path << std::endl;
                        continue;
                    }
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::fromROSMsg(*pc_msg, *pointcloud);
                    for (size_t i = 0; i < pointcloud->points.size (); ++i){
                        bin_file.write((char*)&pointcloud->points[i].x,3*sizeof(float)); 
                        bin_file.write((char*)&pointcloud->points[i].intensity,sizeof(float));
                    }
                    bin_file.close();
                    std::cout << "Save point cloud "<< to_string(pc_msg->header.stamp.toSec())+".bin in" << save_root_path+"pointcloud/" << std::endl;
                }
            }
            f_save_timestamp.close();
        }

        void save_RTK(){
            Eigen::Vector3d lla0;   // first longtitude latitude altitude
            int first_flag = 0;
            Eigen::Vector3d enu_last;
            Eigen::Vector3d enu_last_last;
            double noise_thre_last = 1.0;
            double total_travel_distance = 0.0;
            double first_time = 0.0;
            int RTK_count = 0;
            std::vector<std::pair<double, Eigen::Vector3d>> RTK_raw_data_with_time;
            std::vector<std::pair<double, Eigen::Vector3d>> RTK_fix_data_with_time;
            std::vector<std::pair<double, Eigen::Vector3d>> RTK_apc_data_with_time;
            std::unordered_set<int> abnormal_values;
            
            std::ofstream f_save_RTK_raw_data;
            std::ofstream f_save_groundtruth_raw_frequency;
            std::ofstream f_save_groundtruth_pc_frequency;
            f_save_RTK_raw_data.open(save_root_path+"RTK/"+"RTK_raw_data.txt", std::fstream::out);
            f_save_groundtruth_raw_frequency.open(save_root_path+"RTK/"+"gt_raw_frequency.txt", std::fstream::out);
            f_save_groundtruth_pc_frequency.open(save_root_path+"RTK/"+"gt_pc_frequency.txt", std::fstream::out);

            if (f_save_RTK_raw_data.is_open() && f_save_groundtruth_raw_frequency.is_open() && f_save_groundtruth_pc_frequency.is_open()){
                std::cerr << "All RTK files open successfully......" << std::endl << std::endl;
            }
            else{
                std::cerr << "Some files are not open......" << std::endl << std::endl;
            }
            rosbag::View RTK_view(bag, rosbag::TopicQuery(RTK_topic));
            if (RTK_view.size() == 0){
                ROS_WARN("Not found RTK data! Skip it!");
                return;
            }
            RTK_exist = 1;
            
            // get raw RTK data
            for (rosbag::View::iterator it = RTK_view.begin(); it != RTK_view.end(); ++it){
                sensor_msgs::NavSatFixConstPtr RTK_msg = it->instantiate<sensor_msgs::NavSatFix>();
                if (RTK_msg != NULL){
                    Eigen::Vector3d lla;
                    double time = RTK_msg->header.stamp.toSec();
                    if (first_time == 0){
                        first_time = time;
                    }
                    lla.x() = RTK_msg->longitude;
                    lla.y() = RTK_msg->latitude;
                    lla.z() = RTK_msg->altitude;
                    if (first_flag == 0){
                        lla0 = lla;
                        first_flag = 1;
                    }
                    Eigen::Vector3d ecef = lla2ecef(lla);
                    Eigen::Vector3d oxyz = lla2ecef(lla0);
                    double x, y, z; // save the x y z in ecef
                    x = ecef.x();
                    y = ecef.y();
                    z = ecef.z();
                    double ox, oy, oz;
                    ox = oxyz.x(); // obtain x in ecef
                    oy = oxyz.y(); // obtain y in ecef
                    oz = oxyz.z(); // obtain z in ecef
                    double dx, dy, dz;
                    dx = x - ox;
                    dy = y - oy;
                    dz = z - oz;
                    double lonDeg, latDeg; // save the origin lon alt in lla
                    lonDeg = lla0.x();
                    latDeg = lla0.y();
                    double lon = lonDeg * deg2rad;
                    double lat = latDeg * deg2rad;
                    Eigen::Vector3d enu;
                    enu.x() = -sin(lon) * dx + cos(lon) * dy;
                    enu.y() = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
                    enu.z() = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
                    // identify abnormal value
                    if (RTK_count >= 8){
                        for (int j = 1; j <= 8; j++){
                            if ((abs(enu.x() - RTK_raw_data_with_time[RTK_count-j].second.x()) > j * noise_thre_last ||
                                abs(enu.y() - RTK_raw_data_with_time[RTK_count-j].second.y()) > j * noise_thre_last ||
                                abs(enu.z() - RTK_raw_data_with_time[RTK_count-j].second.z()) > j * noise_thre_last)){
                                    if (abnormal_values.count(RTK_count-j) == 0){
                                        abnormal_values.insert(RTK_count);
                                        break;
                                    }
                                    else{
                                        continue;
                                    }
                                }
                            else if (abnormal_values.count(RTK_count-j) == 0){
                                // normal
                                break;
                            }
                        }
                        // ROS_WARN("Filter Start!!!");
                        // std::cout << RTK_count << std::endl;
                    }
                    std::cout << "RTK time: " << to_string(time) << " x: " << enu.x() << " y: " << enu.y() << " z: " << enu.z() << std::endl;
                    f_save_RTK_raw_data << std::fixed << std::setprecision(6) << time << " " <<
                        std::setprecision(9) << enu.x() << " " << enu.y() << " " << enu.z() << " " 
                        << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                    
                    RTK_raw_data_with_time.push_back(std::make_pair(time, enu));
                    RTK_count++;
                }
            }

            // interpolation （fix RTK dara at RTK raw frequency)
            static Eigen::Vector3d pose_last;
            for (long unsigned int i = 0; i < RTK_raw_data_with_time.size(); i++){
                double time = RTK_raw_data_with_time[i].first;
                Eigen::Vector3d pose = RTK_raw_data_with_time[i].second;
                if (abnormal_values.count(i) == 0 || i == 0 || i == RTK_raw_data_with_time.size() - 1){
                    f_save_groundtruth_raw_frequency << std::fixed << std::setprecision(6) << time << " " <<
                        std::setprecision(9) << pose.x() << " " << pose.y() << " " << pose.z() << " " 
                        << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                    // ROS_INFO("Add normal data!!!");
                }
                else{
                    // interpolation pose
                    int frontid = i + 1, backid = i - 1;
                    while (abnormal_values.count(frontid) != 0){
                        frontid++;
                    }
                    double time_front = RTK_raw_data_with_time[frontid].first;
                    double time_back = RTK_raw_data_with_time[backid].first;
                    Eigen::Vector3d pose_front = RTK_raw_data_with_time[frontid].second;
                    Eigen::Vector3d pose_back = RTK_raw_data_with_time[backid].second;

                    double dt_1 = time - time_back;
                    double dt_2 = time_front - time;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    pose = w1 * pose_back + w2 * pose_front;
                    RTK_raw_data_with_time[i].second = pose;
                    f_save_groundtruth_raw_frequency << std::fixed << std::setprecision(6) << time << " " <<
                        std::setprecision(9) << pose.x() << " " << pose.y() << " " << pose.z() << " " 
                        << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                    // ROS_ERROR("Fix abnormal data!!!");
                    // std::cout << i << std::endl;
                }
                if (i == 0){
                    pose_last = pose;
                }
                total_travel_distance += (pose - pose_last).norm();
                pose_last = pose;
                RTK_fix_data_with_time.push_back(std::make_pair(time, pose));

                std::cout << "Time: " << std::to_string(time - first_time) << " s" << std::endl;
                std::cout << "Total Travel Distance is: " << total_travel_distance << " m" << std::endl;
                std::cout << "RTK pose: x " << pose.x() << " y " << pose.y() << " z " << pose.z() << std::endl;
                // std::cout << "\033[2J \033[0m" << std::endl;
            }
            
            // interpolation （fix RTK dara at pointcloud frequency)
            int frontid = 0, backid = 0;
            double time_front = RTK_fix_data_with_time[frontid].first;
            double time_back = RTK_fix_data_with_time[backid].first;
            Eigen::Vector3d pose_front = RTK_fix_data_with_time[frontid].second;
            Eigen::Vector3d pose_back = RTK_fix_data_with_time[backid].second;
            for (long unsigned int i = 0; i < timestamps.size(); ++i){
                double pc_time = timestamps[i];
                // if the time difference between RTK pose and pointcloud pose is small enough
                if (abs(pc_time - time_front)<0.0001){
                    RTK_apc_data_with_time.push_back(std::make_pair(time_front, pose_front));
                    f_save_groundtruth_pc_frequency << std::fixed << std::setprecision(6) << time_front << " " <<
                            std::setprecision(9) << pose_front.x() << " " << pose_front.y() << " " << pose_front.z() << " " 
                            << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                    if (i == 0){
                        pose_last = pose_front;
                    }
                    pose_last = pose_front;
                }
                else{
                    if (pc_time < time_back){
                        RTK_apc_data_with_time.push_back(std::make_pair(pc_time, pose_back));
                        f_save_groundtruth_pc_frequency << std::fixed << std::setprecision(6) << pc_time << " " <<
                            std::setprecision(9) << pose_back.x() << " " << pose_back.y() << " " << pose_back.z() << " " 
                            << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                        if (i == 0){
                            pose_last = pose_back;
                        }
                        pose_last = pose_back;
                        continue;
                    }
                    // if pointcloud pose time is large than RTK pose time, look backwards
                    while (pc_time > time_front){
                        frontid++;
                        time_front = RTK_fix_data_with_time[frontid].first;
                        pose_front = RTK_fix_data_with_time[frontid].second;
                    }
                    
                    if (abs(pc_time - time_front)<0.0001){
                        RTK_apc_data_with_time.push_back(std::make_pair(time_front, pose_front));
                        f_save_groundtruth_pc_frequency << std::fixed << std::setprecision(6) << time_front << " " <<
                            std::setprecision(9) << pose_front.x() << " " << pose_front.y() << " " << pose_front.z() << " " 
                            << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                        if (i == 0){
                            pose_last = pose_front;
                        }
                        pose_last = pose_front;
                    }
                    else{
                        // if still not satisfied, use the front and back odometry pose interpolation
                        backid = frontid - 1;
                        time_back = RTK_fix_data_with_time[backid].first;
                        pose_back = RTK_fix_data_with_time[backid].second;

                        Eigen::Vector3d pose_inter;// interpolation pose
                        double dt_1 = pc_time - time_back;
                        double dt_2 = time_front - pc_time;
                        ROS_ASSERT(dt_1 >= 0);
                        ROS_ASSERT(dt_2 >= 0);
                        ROS_ASSERT(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        pose_inter = w1 * pose_back + w2 * pose_front;

                        RTK_apc_data_with_time.push_back(std::make_pair(pc_time, pose_inter));
                        f_save_groundtruth_pc_frequency << std::fixed << std::setprecision(6) << pc_time << " " <<
                            std::setprecision(9) << pose_inter.x() << " " << pose_inter.y() << " " << pose_inter.z() << " " 
                            << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                        if (i == 0){
                            pose_last = pose_inter;
                        }
                    total_travel_distance += (pose_inter - pose_last).norm();
                    pose_last = pose_inter;
                    }
                }
                // std::cout << "Total Travel Distance is: " << total_travel_distance << " m" << std::endl;
            }
            f_save_RTK_raw_data.close();
            f_save_groundtruth_raw_frequency.close();
            f_save_groundtruth_pc_frequency.close();
        }

        void save_image(){
            std::ofstream f_save_color, f_save_depth;
            f_save_color.open(save_root_path+"image/color.txt", std::fstream::out);
            f_save_depth.open(save_root_path+"image/depth.txt", std::fstream::out);
            if (f_save_color.is_open() && f_save_depth.is_open()){
                std::cerr << "color.txt and depth.txt file opens successfully......" << std::endl << std::endl;
            }
            else{
                std::cerr << "color.txt or depth.txt file opens failed......" << std::endl << std::endl;
            }

            rosbag::View image_color_view(bag, rosbag::TopicQuery(Camera_topic));
            rosbag::View image_depth_view(bag, rosbag::TopicQuery(Depth_topic));
            if (image_color_view.size() == 0 || image_depth_view.size() == 0){
                ROS_WARN("Not found color image data or depth data! Skip it!");
                return;
            }
            color_exist = 1;
            depth_exist = 1;
            for (rosbag::View::iterator it = image_color_view.begin(); it != image_color_view.end(); ++it){
                sensor_msgs::ImageConstPtr image_color_msg = it->instantiate<sensor_msgs::Image>();
                if (image_color_msg != NULL){
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
                    //save results
                    cv::imwrite(save_root_path+"image/color/"+to_string(image_color_msg->header.stamp.toSec())+".png", cv_ptr->image);
                    std::cout << "save color image "<< to_string(image_color_msg->header.stamp.toSec())+".png in" << save_root_path+"image/color/" << std::endl;
                    f_save_color << std::fixed << image_color_msg->header.stamp.toSec() << " color/" << to_string(image_color_msg->header.stamp.toSec())+".png" << std::endl;
                }
            }
            for (rosbag::View::iterator it = image_depth_view.begin(); it != image_depth_view.end(); ++it){
                sensor_msgs::ImageConstPtr image_depth_msg = it->instantiate<sensor_msgs::Image>();
                if (image_depth_msg != NULL){
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_depth_msg, image_depth_msg->encoding);
                    //save results
                    cv::imwrite(save_root_path+"image/depth/"+to_string(image_depth_msg->header.stamp.toSec())+".png", cv_ptr->image);
                    std::cout << "save depth image "<< to_string(image_depth_msg->header.stamp.toSec())+".png in" << save_root_path+"image/depth/" << std::endl;
                    f_save_depth << std::fixed << image_depth_msg->header.stamp.toSec() << " depth/" << to_string(image_depth_msg->header.stamp.toSec())+".png" << std::endl;
                }
            }
        }

        void save_IMU(){
            std::ofstream f_save_IMU_data(save_root_path+"imu/data.csv", ios::out);
            std::ofstream f_save_MAG_data(save_root_path+"imu/mag.csv", ios::out);
            if (f_save_IMU_data.is_open() && f_save_MAG_data.is_open()){
                std::cerr << "IMU files open successfully......" << std::endl << std::endl;
            }
            else{
                std::cerr << "IMU files open failed......" << std::endl << std::endl;
            }
            rosbag::View imu_view(bag, rosbag::TopicQuery(IMU_topic));
            if (imu_view.size() == 0){
                ROS_WARN("Not found IMU data! Skip it!");
                return;
            }
            imu_exist = 1;
            f_save_IMU_data << "time" << ',' << "ax" << ',' << "ay" << ',' << "az" << ',' << "gx" << ',' << "gy" << ',' << "gz" << std::endl;
            for (rosbag::View::iterator it = imu_view.begin(); it != imu_view.end(); ++it){
                sensor_msgs::ImuConstPtr imu_msg = it->instantiate<sensor_msgs::Imu>();
                if (imu_msg != NULL){
                    f_save_IMU_data << imu_msg->header.stamp.toSec() << ',' << imu_msg->linear_acceleration.x 
                    << ',' << imu_msg->linear_acceleration.y << ',' << imu_msg->linear_acceleration.z << ','
                    << imu_msg->angular_velocity.x << ',' << imu_msg->angular_velocity.y << ',' << imu_msg->angular_velocity.z << std::endl;
                    std::cout << "save IMU data "<< to_string(imu_msg->header.stamp.toSec())+" in" << save_root_path+"imu/data.csv" << std::endl;
                }
            }
            f_save_IMU_data.close();
            rosbag::View mag_view(bag, rosbag::TopicQuery(Mag_topic));
            if (mag_view.size() == 0){
                ROS_WARN("Not found MAG data! Skip it!");
                return;
            }
            mag_exist = 1;
            f_save_MAG_data << "time" << ',' << "ax" << ',' << "ay" << ',' << "az" << std::endl;
            for (rosbag::View::iterator it = mag_view.begin(); it != mag_view.end(); ++it){
                sensor_msgs::MagneticFieldConstPtr mag_msg = it->instantiate<sensor_msgs::MagneticField>();
                if (mag_msg != NULL){
                    platform = 2;
                    f_save_MAG_data << mag_msg->header.stamp.toSec() << ',' << mag_msg->magnetic_field.x 
                    << ',' << mag_msg->magnetic_field.y << ',' << mag_msg->magnetic_field.z << std::endl;
                    std::cout << "save MAG data "<< to_string(mag_msg->header.stamp.toSec())+" in" << save_root_path+"imu/mag.csv" << std::endl;
                }
                geometry_msgs::Vector3StampedConstPtr mag_msg2 = it->instantiate<geometry_msgs::Vector3Stamped>();
                if (mag_msg2 != NULL){
                    platform = 1;
                    f_save_MAG_data << mag_msg2->header.stamp.toSec() << ',' << mag_msg2->vector.x 
                    << ',' << mag_msg2->vector.y << ',' << mag_msg2->vector.z << std::endl;
                    std::cout << "save MAG data "<< to_string(mag_msg2->header.stamp.toSec())+" in" << save_root_path+"imu/mag.csv" << std::endl;
                }
            }
            f_save_IMU_data.close();
            f_save_MAG_data.close();
        }

        void save_calibration(){
            YAML::Node calibration;
            if(platform == 1){
                calibration["robot"] = "4_wheeled_robot";

                calibration["imu"]["acc_cov"] = 1.2561445995419982e-02;
                calibration["imu"]["gyr_cov"] = 9.3313968940900488e-05;
                calibration["imu"]["b_acc_cov"] = 1.2867571334272321e-04;
                calibration["imu"]["b_gyr_cov"] = 1.2867571334272321e-04;
                
                calibration["camera"]["intrinsic"]["fx"] = 382.988524856419;
                calibration["camera"]["intrinsic"]["fy"] = 382.284956247964;
                calibration["camera"]["intrinsic"]["u0"] = 313.870579957838;
                calibration["camera"]["intrinsic"]["v0"] = 243.984540052013;

                calibration["camera"]["distcoeff"]["k1"] = -0.0505434532886371;
                calibration["camera"]["distcoeff"]["k2"] = 0.0373690602433171;

                calibration["extrinsic"]["lidar_to_imu"]["roll"] = -0.132824;
                calibration["extrinsic"]["lidar_to_imu"]["pitch"] = -1.202763;
                calibration["extrinsic"]["lidar_to_imu"]["yaw"] = -1.853268;
                calibration["extrinsic"]["lidar_to_imu"]["x"] = -0.019070;
                calibration["extrinsic"]["lidar_to_imu"]["y"] = 0.004500;
                calibration["extrinsic"]["lidar_to_imu"]["z"] = 0.057140;

                calibration["extrinsic"]["lidar_to_robot"]["roll"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["pitch"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["yaw"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["x"] = 0.42;
                calibration["extrinsic"]["lidar_to_robot"]["y"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["z"] = 0.34;

                calibration["extrinsic"]["lidar_to_camera"]["roll"] = -89.258713;
                calibration["extrinsic"]["lidar_to_camera"]["pitch"] = 0.568155;
                calibration["extrinsic"]["lidar_to_camera"]["yaw"] = -92.129120;
                calibration["extrinsic"]["lidar_to_camera"]["x"] = -0.006783;
                calibration["extrinsic"]["lidar_to_camera"]["y"] = -0.103978;
                calibration["extrinsic"]["lidar_to_camera"]["z"] = -0.153868;
                
            }
            else if(platform == 2){
                calibration["robot"] = "6_wheeled_robot";

                calibration["imu"]["acc_cov"] = 1.1165384285748590e-02;
                calibration["imu"]["gyr_cov"] = 1.0218428289100280e-02;
                calibration["imu"]["b_acc_cov"] = 1.6554947142117672e-04;
                calibration["imu"]["b_gyr_cov"] = 9.1489720415354302e-05;

                calibration["camera"]["intrinsic"]["fx"] = 647.798328608129;
                calibration["camera"]["intrinsic"]["fy"] = 648.749498857231;
                calibration["camera"]["intrinsic"]["u0"] = 634.847217797169;
                calibration["camera"]["intrinsic"]["v0"] = 369.621588030686;

                calibration["camera"]["distcoeff"]["k1"] = -0.0435054940545947;
                calibration["camera"]["distcoeff"]["k2"] = 0.0452193980011014;

                calibration["extrinsic"]["lidar_to_imu"]["roll"] = 179.603;
                calibration["extrinsic"]["lidar_to_imu"]["pitch"] = -179.624;
                calibration["extrinsic"]["lidar_to_imu"]["yaw"] = -179.937;
                calibration["extrinsic"]["lidar_to_imu"]["x"] = 0.76802;
                calibration["extrinsic"]["lidar_to_imu"]["y"] = -0.11605;
                calibration["extrinsic"]["lidar_to_imu"]["z"] = 0.27720;

                calibration["extrinsic"]["lidar_to_robot"]["roll"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["pitch"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["yaw"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["x"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["y"] = 0.0;
                calibration["extrinsic"]["lidar_to_robot"]["z"] = 0.45;

                calibration["extrinsic"]["lidar_to_camera"]["roll"] = -88.999547;
                calibration["extrinsic"]["lidar_to_camera"]["pitch"] = 0.130808;
                calibration["extrinsic"]["lidar_to_camera"]["yaw"] = -89.668972;
                calibration["extrinsic"]["lidar_to_camera"]["x"] = -0.012008;
                calibration["extrinsic"]["lidar_to_camera"]["y"] = -0.129630;
                calibration["extrinsic"]["lidar_to_camera"]["z"] = -0.044118;
        
            }
            calibration["topic"]["RTK"] = RTK_exist == 1 ? RTK_topic : "None";
            calibration["topic"]["color_image"] = color_exist == 1 ? Camera_topic : "None";
            calibration["topic"]["depth_image"] = depth_exist == 1 ? Depth_topic : "None";
            calibration["topic"]["point_cloud"] = pc_exist == 1 ? LiDAR_topic : "None";
            calibration["topic"]["imu"] = imu_exist == 1 ? IMU_topic : "None";
            calibration["topic"]["mag"] = mag_exist == 1 ? Mag_topic : "None";
            
            std::ofstream f_calibration(save_root_path + "calibration/calibration.yaml");
            f_calibration << calibration;
            f_calibration.close();

        }

        void run(){
            save_pointcloud();
            save_RTK();
            save_image();
            save_IMU();
            save_calibration();
        }
};

int main(int argc, char **argv){
	ros::init(argc, argv, "bag2file");

    Bag2File b2f;
    b2f.run();

	return 0;
}
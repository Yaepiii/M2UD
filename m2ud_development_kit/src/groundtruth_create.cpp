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

class GTCreate{
    public:
        ros::NodeHandle nh;
        
        std::ofstream f_save_RTK_raw_data;
        std::ofstream f_save_groundtruth_raw_frequency;
        std::ofstream f_save_pc_timestamp;
        std::ofstream f_save_groundtruth_pc_frequency;
        Eigen::Vector3d lla0;   // first longtitude latitude altitude
        int flag = 0;
        Eigen::Vector3d enu_last;
        Eigen::Vector3d enu_last_last;

        double noise_thre_last = 0.7;

        ros::Publisher RTK_raw_puber;
        ros::Publisher RTK_fix_puber;
        std::string rosbag_path;
        std::string rosbag_file;
        std::string save_root_path;
        std::string RTK_topic;

        double total_travel_distance;
        double first_time = 0;

        int RTK_count = 0;
        std::vector<std::pair<double, Eigen::Vector3d>> RTK_raw_data_with_time;
        std::vector<std::pair<double, Eigen::Vector3d>> RTK_fix_data_with_time;
        std::vector<std::pair<double, Eigen::Vector3d>> RTK_apc_data_with_time;
        std::unordered_set<int> abnormal_values;
        std::vector<double> pc_times;
        rosbag::Bag bag;

        nav_msgs::Path RTK_path;

        GTCreate(){
            ros::param::param<std::string>("rosbag_path", rosbag_path, "The root directory where you bag file");
            ros::param::param<std::string>("rosbag_file", rosbag_file, "The bag file");
            ros::param::param<std::string>("RTK_topic", RTK_topic, "The RTK topic");
            save_root_path = rosbag_path + "GTCreate/";
            int32_t mkdir_status = openDirAndMkdir(save_root_path.c_str());
            if (mkdir_status == -1){
                std::cerr << "Mkdir failed ......" << std::endl << std::endl;
                ROS_BREAK();
            }
            f_save_RTK_raw_data.open(save_root_path + "RTK_raw_data.txt", std::fstream::out);
            f_save_groundtruth_raw_frequency.open(save_root_path+"gt_raw_frequency.txt", std::fstream::out);
            f_save_pc_timestamp.open(save_root_path+"pc_timestamp.txt", std::fstream::out);
            f_save_groundtruth_pc_frequency.open(save_root_path+"gt_pc_frequency.txt", std::fstream::out);
            // open rosbag file
            bag.open(rosbag_path+rosbag_file, rosbag::bagmode::Read);

            RTK_raw_puber = nh.advertise<nav_msgs::Path>("/RTK_raw_path", 10);
            RTK_fix_puber = nh.advertise<nav_msgs::Path>("/RTK_fixed_path", 10);
            if (f_save_RTK_raw_data.is_open() && f_save_groundtruth_raw_frequency.is_open() && f_save_pc_timestamp.is_open() && f_save_groundtruth_pc_frequency.is_open()){
                std::cerr << "All files open successful ......" << std::endl << std::endl;
            }
            else{
                std::cerr << "Some files are not open ......" << std::endl << std::endl;
                ROS_BREAK();
            }
        }
        ~GTCreate(){
            f_save_RTK_raw_data.close();
            f_save_groundtruth_raw_frequency.close();
            f_save_pc_timestamp.close();
            f_save_groundtruth_pc_frequency.close();
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

        void pubRTKPath(double time, Eigen::Vector3d RTK_pose, ros::Publisher& pub){
            geometry_msgs::PoseStamped RTK_pose_tmp;
            RTK_pose_tmp.header.stamp = ros::Time().fromSec(time);
            RTK_pose_tmp.header.frame_id = "world";
            RTK_pose_tmp.pose.orientation.x = 0;
            RTK_pose_tmp.pose.orientation.y = 0;
            RTK_pose_tmp.pose.orientation.z = 0;
            RTK_pose_tmp.pose.orientation.w = 1;
            RTK_pose_tmp.pose.position.x = RTK_pose.x();
            RTK_pose_tmp.pose.position.y = RTK_pose.y();
            RTK_pose_tmp.pose.position.z = RTK_pose.z();
            RTK_path.header.stamp = RTK_pose_tmp.header.stamp;
            RTK_path.header.frame_id = "world";
            RTK_path.poses.push_back(RTK_pose_tmp);
            pub.publish(RTK_path);
        }

        void run(){
            rosbag::View RTK_view(bag, rosbag::TopicQuery(RTK_topic));
            if (RTK_view.size() == 0){
                ROS_WARN("Not found RTK data! Exit!");
                return;
            }
            // get raw RTK data
            for (rosbag::View::iterator it = RTK_view.begin(); it != RTK_view.end(); ++it){
                sensor_msgs::NavSatFixConstPtr RTK_msg = it->instantiate<sensor_msgs::NavSatFix>();
                // if(RTK_count < 5){
                //     RTK_count++;
                //     continue;
                // }
                if (RTK_msg != NULL){
                    Eigen::Vector3d lla;
                    double time = RTK_msg->header.stamp.toSec();
                    if (first_time == 0){
                        first_time = time;
                    }
                    lla.x() = RTK_msg->longitude;
                    lla.y() = RTK_msg->latitude;
                    lla.z() = RTK_msg->altitude;
                    if (flag == 0){
                        lla0 = lla;
                        flag = 1;
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
                    }
                    std::cout << "RTK time: " << to_string(time) << " x: " << enu.x() << " y: " << enu.y() << " z: " << enu.z() << std::endl;
                    f_save_RTK_raw_data << std::fixed << std::setprecision(6) << time << " " <<
                        std::setprecision(9) << enu.x() << " " << enu.y() << " " << enu.z() << " " 
                        << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
                    
                    RTK_raw_data_with_time.push_back(std::make_pair(time, enu));
                    // pubRTKPath(time, enu, RTK_raw_puber);
                    RTK_count++;
                }
            }

            // interpolation （fix RTK dara at RTK raw frequency)
            RTK_path.poses.clear();
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
                }
                pubRTKPath(time, pose, RTK_fix_puber);
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
        
            rosbag::View pc_view(bag, rosbag::TopicQuery("/velodyne_points"));
            for (rosbag::View::iterator it = pc_view.begin(); it != pc_view.end(); ++it){
                sensor_msgs::PointCloud2ConstPtr pc_msg = it->instantiate<sensor_msgs::PointCloud2>();
                if (pc_msg != NULL){
                    pc_times.push_back(pc_msg->header.stamp.toSec());
                    f_save_pc_timestamp << std::fixed << std::setprecision(6) << pc_msg->header.stamp.toSec() << std::endl;
                }
            }
            
            // interpolation （fix RTK dara at pointcloud frequency)
            RTK_path.poses.clear();
            int frontid = 0, backid = 0;
            double time_front = RTK_fix_data_with_time[frontid].first;
            double time_back = RTK_fix_data_with_time[backid].first;
            Eigen::Vector3d pose_front = RTK_fix_data_with_time[frontid].second;
            Eigen::Vector3d pose_back = RTK_fix_data_with_time[backid].second;
            for (long unsigned int i = 0; i < pc_times.size(); ++i){
                double pc_time = pc_times[i];
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
        }
};

int main(int argc, char **argv){
	ros::init(argc, argv, "groundtruth_create");

    GTCreate gc;
    gc.run();

	return 0;
}
/****************************************************************************************
 *
 * Copyright (c) 2024, Shenyang Institute of Automation, Chinese Academy of Sciences
 *
 * Authors: Yanpeng Jia
 * Contact: yaepiii@126.com
 * 
 * Usage: Get pose ground truth from RTK topic and show ground truth trajectory on Rviz
 *
 ****************************************************************************************/
#include "utility.h"

class TimeAlign{
  public:
    ros::NodeHandle nh;

    pcl::PointCloud<PointTypePose>::Ptr ref_poses6D, before_odom_poses6D, after_odom_poses6D;
    int ref_pose_num = 0;
    int odom_pose_num = 0;
    
    std::string groundtruth;
    std::string before_data;
    std::string after_data;

    std::ifstream f_read_ref, f_before_odom;
    std::ofstream f_after_odom;

    TimeAlign(){
    	ros::param::param<std::string>("groundtruth", this->groundtruth, "your calibrated ground truth .txt file");
    	ros::param::param<std::string>("before_data", this->before_data, "pose file before time align");
    	ros::param::param<std::string>("after_data", this->after_data, "pose file after time align to save");
        ref_poses6D.reset(new pcl::PointCloud<PointTypePose>());
        before_odom_poses6D.reset(new pcl::PointCloud<PointTypePose>());
        after_odom_poses6D.reset(new pcl::PointCloud<PointTypePose>());

        f_read_ref.open(this->groundtruth, std::fstream::in);
        if (f_read_ref.is_open()){
            char line[1024];
            while (f_read_ref.getline(line, sizeof(line), '\n')){
                char *p;
                p = strtok(line, " ");
                vector<double> pose;
                while (p != NULL){
                    pose.push_back(atof(p));
                    p = strtok(NULL, " ");
                }
                PointTypePose this_pose6D;
                this_pose6D.time = pose[0];
                this_pose6D.x = pose[1];
                this_pose6D.y = pose[2];
                this_pose6D.z = pose[3];
                this_pose6D.qx = pose[4];
                this_pose6D.qy = pose[5];
                this_pose6D.qz = pose[6];
                this_pose6D.qw = pose[7];
                this_pose6D.intensity = ref_pose_num;
                ref_poses6D->push_back(this_pose6D);
                ref_pose_num++;
            }
        }
        f_read_ref.close();
        printf("reference pose load finish! (%d) \n", ref_pose_num);

        f_before_odom.open(this->before_data, std::fstream::in);
        if (f_before_odom.is_open()){
            char line[1024];
            while (f_before_odom.getline(line, sizeof(line), '\n')){
                char *p;
                p = strtok(line, " ");
                vector<double> pose;
                while (p != NULL){
                    pose.push_back(atof(p));
                    p = strtok(NULL, " ");
                }
                PointTypePose this_pose6D;
                this_pose6D.time = pose[0];
                this_pose6D.x = pose[1];
                this_pose6D.y = pose[2];
                this_pose6D.z = pose[3];
                this_pose6D.qx = pose[4];
                this_pose6D.qy = pose[5];
                this_pose6D.qz = pose[6];
                this_pose6D.qw = pose[7];
                this_pose6D.intensity = odom_pose_num;
                before_odom_poses6D->push_back(this_pose6D);
                odom_pose_num++;
            }
        }
        f_before_odom.close();
        printf("odom pose load finish! (%d) \n", odom_pose_num);

        f_after_odom.open(this->after_data, std::fstream::out);
        if (f_after_odom.is_open()){
            std::cerr << "after time align odom file open successful ......" << std::endl << std::endl;
        }
        else{
            std::cerr << "after time align odom file is not open ......" << std::endl << std::endl;
        }
    }

    ~TimeAlign()
    {   
        int keySize = after_odom_poses6D->size();
        for (int i = 0; i < keySize; ++i){
            PointTypePose keyPose6D = after_odom_poses6D->points[i];
            f_after_odom << std::fixed << keyPose6D.time << " " << keyPose6D.x << " " << keyPose6D.y << " " << keyPose6D.z << " "
                                          << keyPose6D.qx << " " << keyPose6D.qy << " " << keyPose6D.qz << " " << keyPose6D.qw << std::endl;
        }
        f_after_odom.close();
        printf("time align exit!!! \n\n");
    }

    void runTimeAlign()
    {
        int frontid = 0, backid = 0;
        PointTypePose odom_pose6D_front = before_odom_poses6D->points[frontid];
        PointTypePose odom_pose6D_back = before_odom_poses6D->points[backid];
        int start_ref = 0;
        for (int i = 0; i < ref_pose_num; ++i)
        {
            PointTypePose ref_pose6D = ref_poses6D->points[i];
            if (ref_pose6D.time < odom_pose6D_back.time)
                start_ref++;
            else
                break;
        }
        for (int i = start_ref; i < ref_pose_num; ++i){
            PointTypePose ref_pose6D = ref_poses6D->points[i];
            // if the time difference between ground truth pose and odometry pose is small enough
            if (abs(ref_pose6D.time - odom_pose6D_front.time)<0.0001){
                after_odom_poses6D->push_back(odom_pose6D_front);
            }
            else{
                // if grond truth pose time is large than odometry pose time, look backwards
                while (ref_pose6D.time > odom_pose6D_front.time){
                    frontid++;
                    if (frontid >= before_odom_poses6D->points.size())
                        break;
                    odom_pose6D_front = before_odom_poses6D->points[frontid];
                }

                if (frontid >= before_odom_poses6D->points.size())
                    break;
                if (abs(ref_pose6D.time - odom_pose6D_front.time)<0.0001){
                    after_odom_poses6D->push_back(odom_pose6D_front);
                }
                else{
                    // if still not satisfied, use the front and back odometry pose interpolation
                    backid = frontid - 1;
                    odom_pose6D_back = before_odom_poses6D->points[backid];

                    Eigen::Vector3d odom_pose_t_front, odom_pose_t_back, odom_pose_t_inter;
                    Eigen::Quaterniond odom_pose_q_front, odom_pose_q_back, odom_pose_q_inter;
                    odom_pose_t_front.x() = odom_pose6D_front.x; odom_pose_t_front.y() = odom_pose6D_front.y; odom_pose_t_front.z() = odom_pose6D_front.z;
                    odom_pose_q_front.x() = odom_pose6D_front.qx; odom_pose_q_front.y() = odom_pose6D_front.qy; odom_pose_q_front.z() = odom_pose6D_front.qz; odom_pose_q_front.w() = odom_pose6D_front.qw;
                    odom_pose_t_back.x() = odom_pose6D_back.x; odom_pose_t_back.y() = odom_pose6D_back.y; odom_pose_t_back.z() = odom_pose6D_back.z;
                    odom_pose_q_back.x() = odom_pose6D_back.qx; odom_pose_q_back.y() = odom_pose6D_back.qy; odom_pose_q_back.z() = odom_pose6D_back.qz; odom_pose_q_back.w() = odom_pose6D_back.qw;

                    PointTypePose odom_pose6D_inter; // interpolation pose
                    double dt_1 = ref_pose6D.time - odom_pose6D_back.time;
                    double dt_2 = odom_pose6D_front.time - ref_pose6D.time;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    odom_pose_t_inter = w1 * odom_pose_t_back + w2 * odom_pose_t_front;
                    odom_pose_q_inter = odom_pose_q_back.slerp(w2, odom_pose_q_front);
                    odom_pose_q_inter.normalize();
                    odom_pose6D_inter.x = odom_pose_t_inter.x();
                    odom_pose6D_inter.y = odom_pose_t_inter.y();
                    odom_pose6D_inter.z = odom_pose_t_inter.z();
                    odom_pose6D_inter.qx = odom_pose_q_inter.x();
                    odom_pose6D_inter.qy = odom_pose_q_inter.y();
                    odom_pose6D_inter.qz = odom_pose_q_inter.z();
                    odom_pose6D_inter.qw = odom_pose_q_inter.w();
                    odom_pose6D_inter.time = ref_pose6D.time;

                    after_odom_poses6D->push_back(odom_pose6D_inter);
                }
            }
            std::cout << frontid << std::endl;
            std::cout << after_odom_poses6D->points.size() << std::endl;
        }

        std::cout << after_odom_poses6D->points.size() << std::endl;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_align");

    TimeAlign TA;

    TA.runTimeAlign();

    return 0;
}

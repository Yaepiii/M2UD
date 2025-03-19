#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/Float32.h>

using namespace std;

typedef pcl::PointXYZI PointType;

#define R_EARTH 6371393.0 // m
#define tab4 "    " // m
double deg2rad = M_PI / 180.0;
double rad2deg = 180.0 / M_PI;

struct PointXYZIQT{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    double qx;
    double qy;
    double qz;
    double qw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIQT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (float, qx, qx) (float, qy, qy) (float, qz, qz) (float, qw, qw)
                                   (double, time, time))

typedef PointXYZIQT  PointTypePose;

class Utility{
  public:
    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R){
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr * rad2deg;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr){
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) * deg2rad;
        Scalar_t p = ypr(1) * deg2rad;
        Scalar_t r = ypr(2) * deg2rad;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }
};

// return 0： success or exist
// return -1：faild
int32_t openDirAndMkdir(const char* pathname)
{
	int ret = 0;
	DIR * mydir = NULL;
	mydir = opendir(pathname);
	if(mydir == NULL)
	{
		ret = mkdir(pathname, 0755);
		if(ret != 0)
		{
			std::cout << pathname <<"--mkdir failed." << std::endl;
			return -1;
		}
		std::cout << pathname << "--mkdir sucess." << std::endl;
	}
	else
	{
		std::cout << pathname << "--dir exist." << std::endl;
	}
	closedir(mydir);
	return ret;
}

bool cmp(const pair<int, double>& a, const pair<int, double>& b) {
    return a.second < b.second;
}

typedef struct
{
    std_msgs::Header header;
    double t = 0;
    double status = 0;
    double HDOP = 99, VDOP = 99;
    int Num_satellites = 0;
    double longitude = 0, latitude = 0, altitude = 0, direction = 0;

    double lon_bias = 0, lat_bias = 0, alt_bias = 0, dir_bias = 0;

    double roll = 0, pitch = 0, yaw = 0;

    Eigen::Quaterniond q_eigen = Eigen::Quaterniond::Identity(); 
    Eigen::Matrix3d Rwc = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rbi = Eigen::Matrix3d::Identity();
} GNSS_s;

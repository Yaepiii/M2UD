#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <string>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZI PointType;

namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

class ChangePC
{
  public:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;
    
    int N_SCANS = 16;
    std::string cloudTopic = "velodyne_points";


    ChangePC()
    {  
        printf("--scanRegistration: scan line number %d \n", N_SCANS);

        if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
        {
            printf("only support velodyne with 16, 32 or 64 scan line!");
        }

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 100, &ChangePC::laserCloudHandler, this);

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points2", 100);
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {

        pcl::PointCloud<PointType> laserCloudIn;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
        pcl::PointCloud<velodyne_ros::Point>::Ptr laserCloud(new pcl::PointCloud<velodyne_ros::Point>());

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

        bool halfPassed = false;
        int count = cloudSize;
        velodyne_ros::Point point;
        int point_intensity = 0;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        for (int i = 0; i < cloudSize; ++i)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            point.intensity = laserCloudIn.points[i].intensity;
            float verticalAngle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {
                scanID = int ((verticalAngle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 32)
            {
                scanID = int ((verticalAngle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 64)
            {
                if (verticalAngle >= -8.83)
                {
                    scanID = int((2 - verticalAngle) * 3.0 + 0.5);
                }
                else
                {
                    scanID = N_SCANS / 2 + int((-8.83 - verticalAngle) * 2.0 + 0.5);
                }
                if (verticalAngle > 2 || verticalAngle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                ROS_BREAK();
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > startOri + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - startOri > M_PI)
                    halfPassed = true;
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > endOri + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = 0;
            point.ring = scanID;
            point.time = relTime;
            laserCloud->push_back(point);

        }
        // std::cout << "------------------------" << std::endl;
        sensor_msgs::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
        laserCloudOutMsg.header.frame_id = laserCloudMsg->header.frame_id;
        pubLaserCloud.publish(laserCloudOutMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "give_pc_time_and_ring");

    ChangePC cpc;

    ros::spin();
    return 0;
}

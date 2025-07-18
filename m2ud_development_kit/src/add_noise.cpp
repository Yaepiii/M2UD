#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <random>
#include <boost/foreach.hpp>

class NoiseAdder {
private:
    double point_cloud_noise_stddev;
    double imu_linear_noise_stddev;
    double imu_angular_noise_stddev;
    double image_noise_stddev;
    
    std::default_random_engine generator;
    
    std::string input_point_cloud_topic;
    std::string input_imu_topic;
    std::string input_image_topic;
    std::string output_point_cloud_topic;
    std::string output_imu_topic;
    std::string output_image_topic;
    
    std::string input_bag_path;
    std::string output_bag_path;
    
public:
    NoiseAdder(ros::NodeHandle& nh) {
        nh.param<double>("point_cloud_noise_stddev", point_cloud_noise_stddev, 0.01);
        nh.param<double>("imu_linear_noise_stddev", imu_linear_noise_stddev, 0.05);
        nh.param<double>("imu_angular_noise_stddev", imu_angular_noise_stddev, 0.01);
        nh.param<double>("image_noise_stddev", image_noise_stddev, 5.0);
        
        nh.param<std::string>("input_point_cloud_topic", input_point_cloud_topic, "/velodyne_points");
        nh.param<std::string>("input_imu_topic", input_imu_topic, "/imu/data");
        nh.param<std::string>("input_image_topic", input_image_topic, "/camera/image_raw");
        
        nh.param<std::string>("output_point_cloud_topic", output_point_cloud_topic, "/noisy_velodyne_points");
        nh.param<std::string>("output_imu_topic", output_imu_topic, "/noisy_imu/data");
        nh.param<std::string>("output_image_topic", output_image_topic, "/noisy_camera/image_raw");
        
        nh.param<std::string>("input_bag_path", input_bag_path, "input.bag");
        nh.param<std::string>("output_bag_path", output_bag_path, "output.bag");
        
        generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    }
    
    // add gaussian noise to pc
    sensor_msgs::PointCloud2 addNoiseToPointCloud(const sensor_msgs::PointCloud2& cloud) {
        sensor_msgs::PointCloud2 noisy_cloud = cloud;
        
        bool has_x = false, has_y = false, has_z = false;
        int x_offset = -1, y_offset = -1, z_offset = -1;
        
        for (const auto& field : noisy_cloud.fields) {
            if (field.name == "x") { has_x = true; x_offset = field.offset; }
            if (field.name == "y") { has_y = true; y_offset = field.offset; }
            if (field.name == "z") { has_z = true; z_offset = field.offset; }
        }
        
        if (!has_x || !has_y || !has_z) {
            ROS_WARN("PointCloud2 message doesn't have x, y, or z fields. Skipping noise addition.");
            return noisy_cloud;
        }
        
        std::normal_distribution<double> dist(0.0, point_cloud_noise_stddev);
        
        for (size_t i = 0; i < noisy_cloud.width * noisy_cloud.height; ++i) {
            float* x = reinterpret_cast<float*>(&noisy_cloud.data[i * noisy_cloud.point_step + x_offset]);
            float* y = reinterpret_cast<float*>(&noisy_cloud.data[i * noisy_cloud.point_step + y_offset]);
            float* z = reinterpret_cast<float*>(&noisy_cloud.data[i * noisy_cloud.point_step + z_offset]);
            
            *x += dist(generator);
            *y += dist(generator);
            *z += dist(generator);
        }
        
        return noisy_cloud;
    }
    
    // add gaussian noise to imu
    sensor_msgs::Imu addNoiseToImu(const sensor_msgs::Imu& imu) {
        sensor_msgs::Imu noisy_imu = imu;
        
        std::normal_distribution<double> linear_dist(0.0, imu_linear_noise_stddev);
        std::normal_distribution<double> angular_dist(0.0, imu_angular_noise_stddev);
        
        noisy_imu.linear_acceleration.x += linear_dist(generator);
        noisy_imu.linear_acceleration.y += linear_dist(generator);
        noisy_imu.linear_acceleration.z += linear_dist(generator);
        
        noisy_imu.angular_velocity.x += angular_dist(generator);
        noisy_imu.angular_velocity.y += angular_dist(generator);
        noisy_imu.angular_velocity.z += angular_dist(generator);
               
        return noisy_imu;
    }
    
    // add gaussian noise to image
    sensor_msgs::Image addNoiseToImage(const sensor_msgs::Image& image) {
        sensor_msgs::Image noisy_image = image;
        
        std::normal_distribution<double> dist(0.0, image_noise_stddev);
        
        if (image.encoding == "mono8" || image.encoding == "8UC1") {
            for (size_t i = 0; i < noisy_image.data.size(); ++i) {
                int new_val = static_cast<int>(noisy_image.data[i]) + static_cast<int>(dist(generator));
                noisy_image.data[i] = static_cast<uint8_t>(std::max(0, std::min(255, new_val)));
            }
        } else if (image.encoding == "rgb8" || image.encoding == "bgr8" || image.encoding == "8UC3") {
            for (size_t i = 0; i < noisy_image.data.size(); ++i) {
                int new_val = static_cast<int>(noisy_image.data[i]) + static_cast<int>(dist(generator));
                noisy_image.data[i] = static_cast<uint8_t>(std::max(0, std::min(255, new_val)));
            }
        } else if (image.encoding == "32FC1") {
            for (size_t i = 0; i < noisy_image.data.size(); i += sizeof(float)) {
                float* val = reinterpret_cast<float*>(&noisy_image.data[i]);
                *val += dist(generator);
            }
        } else {
            ROS_WARN_STREAM("Unsupported image encoding: " << image.encoding << ". Skipping noise addition.");
        }
        
        return noisy_image;
    }
    
    void process() {
        ROS_INFO("Processing bag file: %s", input_bag_path.c_str());
        ROS_INFO("Output bag file: %s", output_bag_path.c_str());
        
        rosbag::Bag input_bag;
        try {
            input_bag.open(input_bag_path, rosbag::bagmode::Read);
        } catch (rosbag::BagException& e) {
            ROS_ERROR("Failed to open input bag file: %s", e.what());
            return;
        }
        
        rosbag::Bag output_bag;
        try {
            output_bag.open(output_bag_path, rosbag::bagmode::Write);
        } catch (rosbag::BagException& e) {
            ROS_ERROR("Failed to open output bag file: %s", e.what());
            input_bag.close();
            return;
        }
        
        std::vector<std::string> topics;
        topics.push_back(input_point_cloud_topic);
        topics.push_back(input_imu_topic);
        topics.push_back(input_image_topic);
        
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            if (m.getTopic() == input_point_cloud_topic || ("/" + m.getTopic() == input_point_cloud_topic)) {
                sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
                // PC
                if (cloud != nullptr) {
                    sensor_msgs::PointCloud2 noisy_cloud = addNoiseToPointCloud(*cloud);
                    output_bag.write(output_point_cloud_topic, m.getTime(), noisy_cloud);
                }
            } else if (m.getTopic() == input_imu_topic || ("/" + m.getTopic() == input_imu_topic)) {
                // IMU
                sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
                if (imu != nullptr) {
                    sensor_msgs::Imu noisy_imu = addNoiseToImu(*imu);
                    output_bag.write(output_imu_topic, m.getTime(), noisy_imu);
                }
            } else if (m.getTopic() == input_image_topic || ("/" + m.getTopic() == input_image_topic)) {
                // Image
                sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
                if (image != nullptr) {
                    sensor_msgs::Image noisy_image = addNoiseToImage(*image);
                    output_bag.write(output_image_topic, m.getTime(), noisy_image);
                }
            }
        }
        
        input_bag.close();
        output_bag.close();
        
        ROS_INFO("Processing completed. Output saved to %s", output_bag_path.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_noise");
    ros::NodeHandle nh("~");
    
    NoiseAdder noise_adder(nh);
    noise_adder.process();
    
    return 0;
}
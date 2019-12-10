#include <mat.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/topic.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

int main(int argc, char **argv)
{
    // Debug message
    ROS_INFO("Starting up");

    // Check if there is a path to a dataset
    if(argc < 3) {
        ROS_ERROR("Error please specify a rosbag file");
        ROS_ERROR("Command Example: rosrun bagconvert bagconvert <rosbag> <topic>");
        return EXIT_FAILURE;
    }

    // Startup this node
    ros::init(argc, argv, "bagconvert");
    ros::NodeHandle nh;

    // Parse the input
    string pathBag = argv[1];
    string imuTopic = argv[2];

    // Get path
    boost::filesystem::path p(pathBag);
    string pathParent = p.parent_path().string();
    string pathMat;
    if(!pathParent.empty()) {
        pathMat = pathParent+"/"+p.stem().string()+".mat";
    } else {
        pathMat = p.stem().string()+".mat";
    }

    rosbag::Bag bag;
    bag.open("/home/vance/dataset/rosbags/vio_imu_1.bag", rosbag::bagmode::Read);
    rosbag::View view(bag);


    ROS_INFO("BAG Path is: %s", pathBag.c_str());
    ROS_INFO("MAT Path is: %s", pathMat.c_str());
    ROS_INFO("IMU Topic is: %s", imuTopic.c_str());
    ROS_INFO("Reading in rosbag file...");
    // 注意后面的m.getTopic()不会获取首字符'/'
    if (boost::starts_with(imuTopic, "/"))
        imuTopic.erase(0 ,1);

    // Create the matlab mat file
    MATFile *pmat = matOpen(pathMat.c_str(), "w");
    if (pmat == nullptr) {
        ROS_ERROR("Error could not create the mat file");
        return(EXIT_FAILURE);
    }

    // Our data vector
    vector<double> dataIMU = vector<double>();

    // Step through the rosbag and send to algo methods
    for (const rosbag::MessageInstance& m : view) {
        // Handle IMU message
//        ROS_INFO("Get a topic: %s", m.getTopic().c_str());
        sensor_msgs::Imu::ConstPtr s1 = m.instantiate<sensor_msgs::Imu>();
        if (s1 != nullptr && m.getTopic() == imuTopic) {
            dataIMU.push_back(m.getTime().toSec());
            dataIMU.push_back(s1->linear_acceleration.x);
            dataIMU.push_back(s1->linear_acceleration.y);
            dataIMU.push_back(s1->linear_acceleration.z);
            dataIMU.push_back(s1->angular_velocity.x);
            dataIMU.push_back(s1->angular_velocity.y);
            dataIMU.push_back(s1->angular_velocity.z);
        }
    }

    ROS_INFO("Done processing bag");
    ROS_INFO("Size of IMU message: %ld", dataIMU.size());


    return EXIT_SUCCESS;
}

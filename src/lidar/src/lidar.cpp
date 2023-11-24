#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh("~");

    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, LidarCallback);
    ros::Rate r(30);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float fMidDist = msg->ranges[719];
    ROS_INFO("Front distance ranges[719] = %f m", fMidDist);
}
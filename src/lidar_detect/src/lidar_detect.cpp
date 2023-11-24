#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

double linear_x;
double linear_y;
double linear_z;
double angular_x;
double angular_y;
double angular_z;
double obstacledist;

geometry_msgs::Twist lidar_vel;
ros::Publisher vel_pub;

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_node");
    ros::NodeHandle nh("~");

    // 从参数服务器获取初始速度参数
    nh.param("linear_x", linear_x, 0.1);
    nh.param("linear_y", linear_y, 0.0);
    nh.param("linear_z", linear_z, 0.0);
    nh.param("angular_x", angular_x, 0.0);
    nh.param("angular_y", angular_y, 0.0);
    nh.param("angular_z", angular_z, 0.0);
    nh.param("obstacle_dist", obstacledist, 1.0);

    lidar_vel.linear.x = linear_x;
    lidar_vel.linear.y = linear_y;
    lidar_vel.linear.z = linear_z;
    lidar_vel.angular.x = angular_x;
    lidar_vel.angular.y = angular_y;
    lidar_vel.angular.z = angular_z;

    ROS_INFO("Linear X: %f", lidar_vel.linear.x);
    ROS_INFO("obstacle_dist: %f", obstacledist);

    vel_pub = nh.advertise<geometry_msgs::Twist>("lidar_vel", 1000);
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, LidarCallback);

    ros::Rate r(30);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float fMidDist = msg->ranges[719];

    // 根据激光雷达数据调整速度
    if (fMidDist > obstacledist) {
        lidar_vel.linear.x = 0.5;  // 如果距离大于1.0米，设置线速度为0.5 m/s
    } else {
        lidar_vel.linear.x = 0.1;  // 否则，设置线速度为0.1 m/s
    }

    ROS_INFO("Front distance ranges[719] = %f m", fMidDist);
    ROS_INFO("Setting linear velocity to: %f m/s", lidar_vel.linear.x);

    // 发布调整后的速度
    vel_pub.publish(lidar_vel);
}

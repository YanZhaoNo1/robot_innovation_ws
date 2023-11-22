#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){

    ros::init(argc,argv,"vel_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist cmd_vel;

    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    cmd_vel.linear.x = nh.getParam("linear_x",linear_x);
    cmd_vel.linear.y = nh.getParam("linear_y",linear_y);
    cmd_vel.linear.z = nh.getParam("linear_z",linear_z);
    cmd_vel.angular.x = nh.getParam("angular_x", angular_x);
    cmd_vel.angular.y = nh.getParam("angular_y", angular_y);
    cmd_vel.angular.z = nh.getParam("angular_z", angular_z);
    // cmd_vel.linear.x = 0.1;
    // cmd_vel.linear.y = 0;
    // cmd_vel.linear.z = 0;
    // cmd_vel.angular.x = 0;
    // cmd_vel.angular.y = 0;
    // cmd_vel.angular.z = 0;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Rate r(30);

    while(ros::ok())
    {
        vel_pub.publish(cmd_vel);
        r.sleep();
    }
}
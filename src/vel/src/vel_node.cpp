#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){

    ros::init(argc,argv,"vel_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;


    // vel_msg.linear.x = nh.getParam("linear_x",0.1);
    // vel_msg.linear.y = nh.getParam("linear_y",0.0);
    // vel_msg.linear.z = nh.getParam("linear_z",0.0);
    // vel_msg.angular.x = nh.getParam("angular_x", 0.0);
    // vel_msg.angular.y = nh.getParam("angular_y", 0.0);
    // vel_msg.angular.z = nh.getParam("angular_z", 0.0);
    vel_msg.linear.x = 0.1;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel_msg",1000);
    ros::Rate r(30);

    while(ros::ok())
    {
        vel_pub.publish(vel_msg);
        r.sleep();
    }
}
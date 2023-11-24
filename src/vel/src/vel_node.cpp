#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){

    ros::init(argc,argv,"vel_node");
    ros::NodeHandle nh("~");
    geometry_msgs::Twist cmd_vel;

    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;


    nh.param("linear_x",linear_x,0.1);
    nh.param("linear_y",linear_y,0.0);
    nh.param("linear_z",linear_z,0.0);
    nh.param("angular_x", angular_x,0.0);
    nh.param("angular_y", angular_y,0.0);
    nh.param("angular_z", angular_z,0.0);

    cmd_vel.linear.x = linear_x;
    cmd_vel.linear.y = linear_y;
    cmd_vel.linear.z = linear_z;
    cmd_vel.angular.x = angular_x;
    cmd_vel.angular.y = angular_y;
    cmd_vel.angular.z = angular_z;

    ROS_INFO("Linear X: %f", cmd_vel.linear.x);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Rate r(30);

    while(ros::ok())
    {
        vel_pub.publish(cmd_vel);
        r.sleep();
    }
}
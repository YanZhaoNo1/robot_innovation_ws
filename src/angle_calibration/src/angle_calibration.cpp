#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"

ros::Publisher vel_pub;
void IMUCallback(sensor_msgs::Imu msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "angle_calibration_node");
    ros::NodeHandle nh("~");

    ros::Subscriber lidar_sub = nh.subscribe("/imu", 10, IMUCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("imu_vel", 1000);
    ros::Rate r(30);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

void IMUCallback(sensor_msgs::Imu msg)
{
    if(msg.orientation_covariance[0]<0)
    return;
    tf::Quaternion quaternion(
                                msg.orientation.x,
                                msg.orientation.y,
                                msg.orientation.z,
                                msg.orientation.w
);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch,yaw);
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    ROS_INFO("roll= %.0f pitch= %.0f yaw= %.0f",roll,pitch,yaw);
    double target_yaw = 90;
    double diff_angle =target_yaw - yaw;
    geometry_msgs::Twist vel_cmd;
    vel_cmd.angular.z=diff_angle*0.01;
    vel_cmd.linear.x = 0.1;
    vel_pub.publish(vel_cmd);
}

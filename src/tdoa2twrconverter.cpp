#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <uwb_driver/UwbRange.h>
#include "localization.h"
#include "cf_msgs/Tdoa.h"

ros::Publisher twr_pub;

double tdoaToDistance(double tdoa)
{
    // Placeholder implementation
    // Convert TDOA to distance based on your system's specifications
    double speed_of_light = 299792458.0;
    double distance = tdoa * speed_of_light; // Adjust as needed
    return distance;
}

void tdoaCallback(const cf_msgs::Tdoa::ConstPtr& msg)
{
    static int seq = 0;

    // Conversion logic from TDOA to TWR distance
    double distance = tdoaToDistance(msg->data); // Implement this function based on your setup

    uwb_driver::UwbRange twr_msg;
    twr_msg.header.seq = seq++;
    twr_msg.header.stamp = ros::Time::now();
    twr_msg.header.frame_id = "uwb";
    twr_msg.requester_id = 200; // Adjust as needed
    twr_msg.requester_idx = msg->idA;
    twr_msg.responder_id = msg->idB;
    twr_msg.responder_idx = 1; // Adjust as needed
    twr_msg.requester_LED_flag = 8; // Adjust as needed
    twr_msg.responder_LED_flag = 8; // Adjust as needed
    twr_msg.noise = 0; // Adjust as needed
    twr_msg.vPeak = 0; // Adjust as needed
    twr_msg.distance = distance;
    twr_msg.distance_err = 0.05; // Adjust as needed
    twr_msg.distance_dot = 0; // Adjust as needed
    twr_msg.distance_dot_err = 0; // Adjust as needed
    twr_msg.antenna = 1; // Adjust as needed
    twr_msg.stopwatch_time = 20; // Adjust as needed
    twr_msg.uwb_time = 0; // Adjust as needed
    // Set responder_location based on your system setup
    twr_msg.responder_location.x = 0; 
    twr_msg.responder_location.y = 0; 
    twr_msg.responder_location.z = 0;

    // Publish the TWR data
    twr_pub.publish(twr_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tdoa2twrconverter");
    ros::NodeHandle nh;

    ros::Subscriber tdoa_sub = nh.subscribe("/tdoa_data", 500, tdoaCallback);
    twr_pub = nh.advertise<uwb_driver::UwbRange>("twr_topic", 300);  // Initialize the publisher


    ros::Rate loop_rate(500); // Publish at 10 Hz

        while (ros::ok())
        {
            ros::spinOnce(); // Process callbacks

            loop_rate.sleep(); // Sleep to control the publishing rate
        }

    return 0;
}
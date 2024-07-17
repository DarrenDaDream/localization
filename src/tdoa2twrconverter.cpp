#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <uwb_driver/UwbRange.h>
#include "localization.h"
#include "cf_msgs/Tdoa.h"
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>

ros::Publisher twr_pub;
std::vector<geometry_msgs::Point> anchors = []{
    std::vector<geometry_msgs::Point> points(8);
    points[0].x = -2.4174718660841163; points[0].y = -4.020796001114614; points[0].z = 0.18179046793237785;
    points[1].x = -2.820490062889947; points[1].y = 3.5250373345173456; points[1].z = 2.5874240006860396;
    points[2].x = 3.4819322476730066; points[2].y = 3.3050399505325867; points[2].z = 0.15447010668018804;
    points[3].x = 3.4507246660737074; points[3].y = -3.7181145718099624; points[3].z = 2.6693201245043428;
    points[4].x = -3.2776160385636026; points[4].y = -3.8689686503275325; points[4].z = 2.6738971667120599;
    points[5].x = 3.2654739320660124; points[5].y = -3.6510796042048415; points[5].z = 0.17524744539737619;
    points[6].x = 3.8321293068358147; points[6].y = 3.6520848542099378; points[6].z = 2.6249273244002822;
    points[7].x = -2.7227724068629255; points[7].y = 3.2190798626426802; points[7].z = 0.15829414514009574;
    return points;
}();

void tdoaCallback(const cf_msgs::Tdoa::ConstPtr& msg)
{
    static int seq = 0;
    int requester_id = msg->idA;
    int responder_id = msg->idB; 

    if (requester_id >= anchors.size() || responder_id >= anchors.size() || requester_id < 0 || responder_id < 0) {
        ROS_WARN("Invalid anchor IDs received: requester_id = %d, responder_id = %d", requester_id, responder_id);
        return;
    }

    double tdoa_distance = msg->data; // Assuming tdoa_distance is the measured TDoA in meters

    // Calculate the direct distances from the tag to each anchor
    double d1 = (sqrt(pow(anchors[requester_id].x, 2) + pow(anchors[requester_id].y, 2) + pow(anchors[requester_id].z, 2)) + tdoa_distance) / 2.0;
    double d2 = d1 - tdoa_distance;

    // TWR distance is the average of d1 and d2
    double twr_distance = (d1 + d2) / 2.0;

    uwb_driver::UwbRange twr_msg;
    twr_msg.header.seq = seq++;
    twr_msg.header.stamp = ros::Time::now();
    twr_msg.header.frame_id = "uwb";
    twr_msg.requester_id = requester_id; // Adjust as needed
    twr_msg.requester_idx = 1;
    twr_msg.responder_id = responder_id;
    twr_msg.responder_idx = 1; // Adjust as needed
    twr_msg.requester_LED_flag = 8; // Adjust as needed
    twr_msg.responder_LED_flag = 8; // Adjust as needed
    twr_msg.noise = 0; // Adjust as needed
    twr_msg.vPeak = 0; // Adjust as needed
    twr_msg.distance = twr_distance;
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
    twr_pub = nh.advertise<uwb_driver::UwbRange>("uwb_endorange_info", 300);  // Initialize the publisher


    ros::Rate loop_rate(500); 

        while (ros::ok())
        {
            ros::spinOnce(); // Process callbacks

            loop_rate.sleep(); // Sleep to control the publishing rate
        }

    return 0;
}
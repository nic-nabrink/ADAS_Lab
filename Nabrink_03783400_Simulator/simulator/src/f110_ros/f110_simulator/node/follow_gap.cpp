#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include <vector>
#include <algorithm> // Required for std::max_element

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#define _USE_MATH_DEFINES
using namespace std;

float angle_min;
float angle_max;
float angle_increment;
float time_increment;
float scan_time;
float range_min;
float range_max;
std::vector<float> ranges;
float intensities;
float speed = 4;
float angle = 0;
float i_s = 0.3;
float c_t = 0.5;
float c_i = -0.17;

void get_angle(std::vector<float> rang){
  float alpha = (2*M_PI)/rang.size(); // index to angle coefficient
  int crop_indice = round(M_PI / (2 * alpha)); // gets only the front of the car
  int upper_limit_index = rang.size() - crop_indice;

  // Initialize maximum value and its index
  float max_value = rang[crop_indice];
  int max_index = crop_indice;
  float min_value = rang[crop_indice];
  int min_index = crop_indice;

  // Iterate through the specified range to find the maximum value and its index
  for (int i = crop_indice + 1; i < upper_limit_index; ++i) {
      if (rang[i] > max_value) {
          max_value = rang[i];
          max_index = i;
      }
      if (rang[i] < min_value) {
          min_value = rang[i];
          min_index = i;
      }
  }

  angle = i_s*((max_index)*alpha-M_PI); //softned input
  angle_min = (min_index)*alpha-M_PI;

  if (min_value < c_t) {
    angle += c_i* angle_min; //anti collision
  }
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ranges = msg->ranges;
    get_angle(ranges);
}



int main(int argc, char *argv[]) {
  ros::init(argc, argv, "follow_gap");
  ros::NodeHandle n;
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/follow_drive", 1);
  ros::Subscriber lidar_sub = n.subscribe("/scan", 1, callback);
  ros::Rate loop_rate(50);

  while(ros::ok()) {
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // Make and publish message
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = angle;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);
    ros::spinOnce(); // look if any callback has to be called
    loop_rate.sleep(); // wait for the end of the period
  }
  return 0;
}

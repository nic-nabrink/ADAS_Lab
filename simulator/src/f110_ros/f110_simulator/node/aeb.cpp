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
#include <std_msgs/Float32.h>

float angle_min;
float angle_max;
float angle_increment;
float time_increment;
float scan_time;
float range_min;
float range_max;
std::vector<float> ranges;
float intensities;
float min_value;
std_msgs::Float32 ttc_speed;

void get_angle(std::vector<float> rang){
  float alpha = (2*M_PI)/rang.size();

  // Initialize maximum value and its index
  int min_index = 0;
  min_value = abs(rang[min_index]/cos((min_index)*alpha-M_PI));

  // Iterate through the specified range to find the maximum value and its index
  for (int i = 0; i < rang.size(); ++i) {
      if (abs(rang[i]/cos(i*alpha-M_PI)) < min_value) {
          min_value = abs((rang[i])/(cos(i*alpha-M_PI)));
      }
  }
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ranges = msg->ranges;
    get_angle(ranges);
    ttc_speed.data = min_value;
}



int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aeb");
  ros::NodeHandle n;
  ros::Publisher command_pub = n.advertise<std_msgs::Float32>("/aeb", 1);
  ros::Subscriber lidar_sub = n.subscribe("/scan", 1, callback);
  ros::Rate loop_rate(50);

  while(ros::ok()) {
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(ttc_speed);
    ros::spinOnce(); // look if any callback has to be called
    loop_rate.sleep(); // wait for the end of the period
  }
  return 0;
}

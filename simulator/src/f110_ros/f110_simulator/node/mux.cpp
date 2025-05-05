#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

float mode = 0;
float speed = 0;
float angle = 0;
float follow_speed = 0;
float follow_angle = 0;
float keyboard_speed = 0;
float keyboard_angle = 0;
float ttc = 1;
int ttc_enable = 0;
float ttc_lt = 0.3;
float ttc_ut = 0.45;

ackermann_msgs::AckermannDriveStamped follow_drive_msg;
ackermann_msgs::AckermannDriveStamped keyboard_drive_msg;

void callback_mode(const std_msgs::Bool::ConstPtr& msg) {
    mode = msg->data; //get the mode
}

void callback_follow(const ackermann_msgs::AckermannDriveStamped::ConstPtr& follow_drive) {
  follow_speed = (follow_drive->drive).speed;
  follow_angle = (follow_drive->drive).steering_angle;
}

void callback_keyboard(const ackermann_msgs::AckermannDriveStamped::ConstPtr& keyboard_drive) {
  if (ttc_enable==0) {
  keyboard_speed = (keyboard_drive->drive).speed;
  keyboard_angle = (keyboard_drive->drive).steering_angle;
  }
  else {
  keyboard_speed = 0;
  keyboard_angle=0;
  }
}

void callback_aeb(const std_msgs::Float32::ConstPtr& ttc_speed) {
  if (speed!= 0) {
  ttc = ttc_speed->data / speed; //divide the ttc_speed by the speed, getting the ttc
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mux");

  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Subscriber follow_sub = n.subscribe("/follow_drive", 1, callback_follow);
  ros::Subscriber keyboard_sub = n.subscribe("/keyboard_drive", 1, callback_keyboard);
  ros::Subscriber mode_sub = n.subscribe("/mode", 1, callback_mode);
  ros::Subscriber aeb_sub = n.subscribe("/aeb", 1, callback_aeb);
  ros::Rate loop_rate(50);

  while(ros::ok()) {
    if (mode == 0 && !ttc_enable) {
      printf("keyboard mode\n");
      speed = keyboard_speed;
      angle = keyboard_angle;
    }
    else if (!ttc_enable) {
      printf("follow mode\n");
      speed = follow_speed;
      angle = follow_angle;
    }
    if (abs(ttc) < ttc_lt && (mode==0)) { //aeb activation algorithm
      speed = -1.8;
      angle = 0;
      ttc_enable = 1;
      mode = 0;
      printf("%f\n",ttc );
    }
    if ((abs(ttc) > ttc_ut) && (ttc_enable)) { //stops at a safe position
        mode = 0;
        speed = 0;
        angle = 0;
        ttc_enable = 0;
        keyboard_speed = 0;
        keyboard_angle=0;
    }

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
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

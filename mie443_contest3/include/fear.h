#ifndef FEAR_H
#define FEAR_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Sound.h>

extern bool fearActive;
extern ros::Time fearStartTime;
extern bool fearSoundPlayed;

void fearCheckCallback(const sensor_msgs::ImageConstPtr& msg);
void handleFearState(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, sound_play::SoundClient &sc, const std::string &path);

#endif
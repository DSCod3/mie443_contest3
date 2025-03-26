#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

extern ros::Time fearStartTime;
extern bool fearActive;
extern bool fearSoundPlayed;

void fearCheckCallback(const geometry_msgs::Twist::ConstPtr& msg);
void handleFearState(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, sound_play::SoundClient &sc, const std::string &path);
#ifndef FEAR_H

#define FEAR_H



#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <kobuki_msgs/Sound.h>

#include <sensor_msgs/Image.h>       // 添加

#include <sound_play/sound_play.h>   // 添加

#include "header.h"                  // 添加状态定义



// 前向声明

extern Status status;                // 添加



extern ros::Time fearStartTime;

extern bool fearActive;

extern bool fearSoundPlayed;



void fearCheckCallback(const geometry_msgs::Twist::ConstPtr& msg);

void handleFearState(geometry_msgs::Twist &vel, 

                    ros::Publisher &vel_pub, 

                    sound_play::SoundClient &sc,  // 修正声明

                    const std::string &path);

#endif






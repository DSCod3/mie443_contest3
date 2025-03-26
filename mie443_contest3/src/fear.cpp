#include <fear.h>
#include "header.h"

extern Status status;
extern bool playingSound;

ros::Time fearStartTime;
bool fearActive = false;
bool fearSoundPlayed = false;
bool isBacking = false;

void fearCheckCallback(const geometry_msgs::Twist::ConstPtr& msg){
    // 检测是否在后退（linear.x < 0）
    if(msg->linear.x < -0.01) {
        if(!isBacking) {
            fearStartTime = ros::Time::now();
            isBacking = true;
        }
        
        // 检查持续时间是否超过3秒
        ros::Duration duration = ros::Time::now() - fearStartTime;
        if(duration.toSec() > 3.0 && status == S_FOLLOW) {
            fearActive = true;
            status = S_FEAR;
        }
    } else {
        isBacking = false;
        if(status == S_FEAR) {
            status = S_FOLLOW;
            fearSoundPlayed = false;
        }
    }
}

void handleFearState(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, sound_play::SoundClient &sc, const std::string &path){
    ROS_INFO("FEAR STATE ACTIVATED");
    
    // 停止机器人并播放声音
    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub.publish(vel);
    
    if(!fearSoundPlayed){
        sc.playWave(path + "Sad_SPBB.wav");
        fearSoundPlayed = true;
    }
    
    // 保持恐惧状态2秒
    ros::Duration fearDuration = ros::Time::now() - fearStartTime;
    if(fearDuration.toSec() > 2.0) {
        fearActive = false;
        status = S_FOLLOW;
    }
}
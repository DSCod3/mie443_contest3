#include <fear.h>
#include <imageTransporter.hpp>
#include <opencv2/opencv.hpp>
#include "header.h"  // 添加

extern Status status;        // 添加
extern bool playingSound;    // 添加

bool fearActive = false;
ros::Time fearStartTime;
bool fearSoundPlayed = false;

void fearCheckCallback(const sensor_msgs::ImageConstPtr& msg){
    // 使用OpenCV检测目标是否突然靠近
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge error: %s", e.what());
        return;
    }

    // 简单示例：检测图像中心区域运动
    static cv::Mat prev_frame;
    if(!prev_frame.empty()){
        cv::Mat diff;
        cv::absdiff(prev_frame, cv_ptr->image, diff);
        double motion = cv::mean(diff)[0];
        
        // 当检测到突然的大幅度运动时触发
        if(motion > 30 && status == S_FOLLOW){ // 阈值需要根据实际情况调整
            fearActive = true;
            fearStartTime = ros::Time::now();
        }
    }
    prev_frame = cv_ptr->image.clone();
}

void handleFearState(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, sound_play::SoundClient &sc, const std::string &path){
    ROS_INFO("FEAR STATE ACTIVATED");
    
    // 后退摇头2秒
    ros::Duration fearDuration = ros::Time::now() - fearStartTime;
    if(fearDuration.toSec() < 2.0){
        vel.linear.x = -0.3;
        vel.angular.z = 1.5;
        vel_pub.publish(vel);
        
        if(!fearSoundPlayed){
            sc.playWave(path + "Sad_SPBB.wav");
            fearSoundPlayed = true;
        }
    } else {
        fearActive = false;
        fearSoundPlayed = false;
        status = S_FOLLOW;
    }
}
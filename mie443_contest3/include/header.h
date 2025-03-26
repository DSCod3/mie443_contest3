#ifndef HEADER_H
#define HEADER_H

#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>
#include <vector>
#include <stdio.h>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eStop.h>

#include <sound_play/sound_play.h>

#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/SensorState.h>



#pragma region Bumper

extern uint8_t bumper[3];

struct BumpersStruct{
    bool leftPressed;
    bool centerPressed;
    bool rightPressed;
    bool anyPressed;
};

extern BumpersStruct bumpers;

#pragma endregion

enum Status {
    S_FOLLOW,
    S_BUMPER,
    S_CLIFF,
    S_MICROPHONE,
    S_PLACEHOLDER
};

extern Status status;


#endif

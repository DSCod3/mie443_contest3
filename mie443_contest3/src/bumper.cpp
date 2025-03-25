#include <bumper.h>

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
BumpersStruct bumpers;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    ROS_INFO("Bumper callback triggered.");

    bumper[msg->bumper] = msg->state;
    bumpers.leftPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    bumpers.centerPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    bumpers.rightPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];

    bumpers.anyPressed = bumpers.leftPressed || bumpers.centerPressed || bumpers.rightPressed;

    // Update Status
    if(bumpers.anyPressed && status == S_FOLLOW){
        status = S_BUMPER;
        ROS_INFO("Bumper callback triggered HIGH.");
    }
    else if (!bumpers.anyPressed && status == S_BUMPER){
        status = S_FOLLOW;
    }

    if(bumpers.anyPressed){
        ROS_INFO("True");
    }
    else{
        ROS_INFO("False");
    }
}

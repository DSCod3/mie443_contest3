#include <cliff.h>

bool cliffActive = false;

void cliffCallback(const kobuki_msgs::SensorState::ConstPtr& msg) {

    cliffActive = (msg->cliff & kobuki_msgs::SensorState::CLIFF_LEFT || msg->cliff & kobuki_msgs::SensorState::CLIFF_CENTRE || msg->cliff & kobuki_msgs::SensorState::CLIFF_RIGHT);

    if (cliffActive) {
        ROS_INFO("Robot off floor.");
        if(status == S_FOLLOW){
            status = S_CLIFF;
        }
    }

    else{
        ROS_INFO("Robot on floor.");
        if(status == S_CLIFF){
            status = S_FOLLOW;
        }
    }
}
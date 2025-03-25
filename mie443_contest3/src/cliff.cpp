#include <cliff.h>

bool cliffActive = false;

void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg) {

    ROS_INFO("Cliff callback triggered.");

    if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {
        cliffActive = true;
        if(status == S_FOLLOW){
            status = S_CLIFF;
        }
        
    }

    else{
        cliffActive = false;

        if(status == S_CLIFF){
            status = S_FOLLOW;
        }
    }
}
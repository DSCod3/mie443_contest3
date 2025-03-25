#include <cliff.h>

bool cliffActive = false;

void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
    if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {
        cliffActive = true;
    }

    else{
        cliffActive = false;
    }
}
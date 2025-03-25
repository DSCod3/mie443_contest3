#ifndef cliffHeader
#define cliffHeader

#include <header.h>

extern bool cliffActive;

void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg);

#endif
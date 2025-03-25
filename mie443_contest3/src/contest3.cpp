#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <bumper.h>
#include <cliff.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
Status status;
bool playingSound;
ros::Time lastFollowTime;  // To track the last follower command

// Update follower callback to refresh the tracking timer.
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    lastFollowTime = ros::Time::now(); // Update last received time
}

void setMovement(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, float lx, float rz){
	vel.angular.z = rz;
	vel.linear.x = lx;
	vel_pub.publish(vel);
	ros::spinOnce();
}

//-------------------------------------------------------------
int main(int argc, char **argv)
{
    status = S_FOLLOW;
    playingSound = false;
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;
    string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    teleController eStop;

    // Publishers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

    // Subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/events/cliff", 10, &cliffCallback);

    // Contest countdown timer
    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t timeReference = 0;

    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    int world_state = 0;
    double angular = 0.2;
    double linear = 0.0;

    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    ros::Duration(0.5).sleep();

    while(ros::ok() && secondsElapsed <= 480){		
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start).count();

        // LOST TRACKING CHECK:
        // If in S_FOLLOW and no follower command received for >3 seconds, switch to lost tracking (S_PLACEHOLDER)
        if(status == S_FOLLOW && (current_time - lastFollowTime).toSec() > 3.0){
            status = S_PLACEHOLDER;
        }
        // If in lost tracking and a recent follower command is received, revert to follow.
        if(status == S_PLACEHOLDER && (current_time - lastFollowTime).toSec() < 1.0){
            status = S_FOLLOW;
            playingSound = false; // reset sound flag
        }

        switch(status){
            case S_FOLLOW:
                playingSound = false;
                vel_pub.publish(follow_cmd);
                break;

            case S_BUMPER:
                ROS_INFO("BUMPER PRESSED EVENT");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Rage.wav");
                }
                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 2){
                    ros::spinOnce();
                    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(ros::Time::now() - start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();
                status = S_FOLLOW;  // Revert to follow after bumper event
                playingSound = false;
                break;

            case S_CLIFF:
                ROS_INFO("CLIFF ACTIVE EVENT");
                setMovement(vel, vel_pub, 0, 0);
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Discontent.wav");
                }
                // When cliff is no longer active, revert to follow.
                if(!cliffActive){
                    status = S_FOLLOW;
                    playingSound = false;
                }
                break;

            case S_MICROPHONE:
                ROS_INFO("MICROPHONE EVENT: INFATUATED");
                setMovement(vel, vel_pub, 0, 0);
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Infatuated.wav");
                }
                // After processing microphone input, revert to follow.
                status = S_FOLLOW;
                playingSound = false;
                break;

            case S_PLACEHOLDER:  // LOST TRACKING state
                ROS_INFO("LOST TRACKING: SEARCHING");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "sad.wav");
                }
                // Search behavior: rotate slowly in place to try to find the person.
                setMovement(vel, vel_pub, 0, 0.5);
                break;
        }

        loop_rate.sleep();
    }

    return 0;
}

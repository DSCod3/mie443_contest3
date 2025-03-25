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
ros::Time lastFollowTime;  // NEW: To track the last time a follower command was received

// Update follower callback to refresh the tracking timer.
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    lastFollowTime = ros::Time::now();
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
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

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

    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);  //-- for Webcam
    // imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8);  //-- for turtlebot Camera
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    int world_state = 0;

    double angular = 0.2;
    double linear = 0.0;

    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    // Initial pause
    ros::Duration(0.5).sleep();

    while(ros::ok() && secondsElapsed <= 480){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();

        // LOST TRACKING CHECK:
        // If in S_FOLLOW and no follower command received for >3 seconds, switch to lost-tracking (S_PLACEHOLDER)
        if(status == S_FOLLOW && (current_time - lastFollowTime).toSec() > 3.0){
            status = S_PLACEHOLDER;
        }

        switch(status){
            case S_FOLLOW:
                ROS_INFO("S_FOLLOW");
                playingSound = false;
                vel_pub.publish(follow_cmd);
                break;

            case S_BUMPER:
                ROS_INFO("BUMPER PRESSED EVENT");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Rage.wav");
                }
                // Move backwards for 2 seconds.
                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 2){
                    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now() - start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }
                // Shaking behavior: alternate left and right turning.
                for (int i = 0; i < 3; i++){
                    // Shake left: turn right (positive angular velocity)
                    timeReference = secondsElapsed;
                    while(secondsElapsed - timeReference < 1){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now() - start).count();
                        setMovement(vel, vel_pub, 0, 0.5);
                    }
                    // Shake right: turn left (negative angular velocity)
                    timeReference = secondsElapsed;
                    while(secondsElapsed - timeReference < 1){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now() - start).count();
                        setMovement(vel, vel_pub, 0, -0.5);
                    }
                }
                // Stop movement after bumper reaction.
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();
                // Optionally, revert to S_FOLLOW after bumper event.
                status = S_FOLLOW;
                playingSound = false;
                break;

            case S_CLIFF:
                ROS_INFO("CLIFF ACTIVE EVENT");
                setMovement(vel, vel_pub, 0, 0);
                // Debounce the cliff sensor.
                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 1){
                    if(status == S_CLIFF){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now() - start).count();
                        timeReference = secondsElapsed;
                    }
                    ros::spinOnce();
                }
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Discontent.wav");
                }
                break;

            case S_MICROPHONE:
                setMovement(vel, vel_pub, 0, 0);
                break;

            case S_PLACEHOLDER:
                ROS_INFO("LOST TRACKING: SEARCHING");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "sad.wav");
                }
                // Search behavior: slowly rotate to find the lost person.
                setMovement(vel, vel_pub, 0, 0.3);
                // If a recent follower command is received, revert to S_FOLLOW.
                if((current_time - lastFollowTime).toSec() < 1.0){
                    status = S_FOLLOW;
                    playingSound = false;
                }
                break;
        }

        loop_rate.sleep();
    }

    return 0;
}

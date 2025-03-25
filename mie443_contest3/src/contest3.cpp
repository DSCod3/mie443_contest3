#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <bumper.h>
#include <cliff.h>
#include <std_msgs/String.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
Status status;
bool playingSound;
bool lostTriggered = false;  // Flag to prevent repeated lost state triggering
ros::Time lastFollowTime;    // Tracks the time of the last follower command

// Follower callback updates the command and resets the lost-tracking flag.
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    lastFollowTime = ros::Time::now();
    lostTriggered = false;  // New follower command resets lost-tracking flag.
}

// Microphone callback listens for the phrase "good robot" to trigger infatuation.
void microphoneCB(const std_msgs::String::ConstPtr& msg) {
    if (msg->data.find("good robot") != std::string::npos) {
        ROS_INFO("Heard 'good robot', entering S_MICROPHONE state.");
        status = S_MICROPHONE;
    }
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

    // Publishers and Subscribers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/events/cliff", 10, &cliffCallback);
    ros::Subscriber mic_sub = nh.subscribe("microphone/input", 10, &microphoneCB);

    // Timer setup
    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t timeReference = 0;

    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);  // For Webcam
    // imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8);  // For Turtlebot Camera
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    int world_state = 0;
    double angular = 0.2;
    double linear = 0.0;
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    
    ros::Duration(0.5).sleep();
    lastFollowTime = ros::Time::now(); // Initialize follow time

    while(ros::ok() && secondsElapsed <= 480){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start).count();

        // Check for lost tracking:
        // If in S_FOLLOW and no new follower command has been received for >3 seconds
        // OR the follow command is essentially zero, then switch to S_PLACEHOLDER.
        if(status == S_FOLLOW && ((current_time - lastFollowTime).toSec() > 3.0 ||
            (fabs(follow_cmd.linear.x) < 0.01 && fabs(follow_cmd.angular.z) < 0.01))){
            status = S_PLACEHOLDER;
        }

        switch(status){
            case S_FOLLOW:
                ROS_INFO("STATE: S_FOLLOW");
                playingSound = false;
                vel_pub.publish(follow_cmd);
                break;

            case S_BUMPER:
                ROS_INFO("STATE: BUMPER PRESSED EVENT");
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
                // Shaking behavior: alternate left/right with increased speed.
                for (int i = 0; i < 3; i++){
                    ROS_INFO("Shaking LEFT");
                    setMovement(vel, vel_pub, 0, 6.0); // 3× faster angular speed (6.0 rad/s)
                    ros::Duration(0.17).sleep();       // Reduced duration (~0.17 sec)
                    ROS_INFO("Shaking RIGHT");
                    setMovement(vel, vel_pub, 0, -6.0);
                    ros::Duration(0.17).sleep();
                }
                // Stop movement after bumper reaction.
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();
                status = S_FOLLOW;  // Revert to follow state.
                playingSound = false;
                break;

            case S_CLIFF:
                ROS_INFO("STATE: CLIFF ACTIVE EVENT");
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
                ROS_INFO("STATE: MICROPHONE EVENT - HEARD GOOD ROBOT (INFATUATED)");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Infatuated.wav");
                }
                setMovement(vel, vel_pub, 0, 0);
                status = S_FOLLOW;  // Revert to follow after processing microphone input.
                playingSound = false;
                break;

            case S_PLACEHOLDER:
                ROS_INFO("STATE: LOST TRACKING - SEARCHING");
                if(!lostTriggered){
                    if(!playingSound){
                        playingSound = true;
                        sc.playWave(path_to_sounds + "sad.wav");
                    }
                    // Search behavior: slowly rotate for 3 seconds.
                    timeReference = secondsElapsed;
                    while(secondsElapsed - timeReference < 3){
                        setMovement(vel, vel_pub, 0, 0.3);
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now() - start).count();
                    }
                    lostTriggered = true;
                } else {
                    ROS_INFO("STATE: STILL LOST TRACKING (NO ACTION)");
                    setMovement(vel, vel_pub, 0, 0);
                }
                break;
        }

        loop_rate.sleep();
    }

    return 0;
}

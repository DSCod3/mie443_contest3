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
int stop_count = 0;
Status status;
bool playingSound = false;

// New flag to ensure lost state isn’t repeatedly triggered
bool lostTriggered = false;

void handleLostTrack(){
    ROS_INFO("Robot lost track.");
    // Implement any additional lost-tracking behavior here.
    // For example, you might rotate slowly to search for the target.
}

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    // Log the received command for debugging.
    ROS_INFO("x, y, z: [%f, %f, %f]", msg.linear.x, msg.linear.y, msg.angular.z);
    // Reset stop_count and lost flag if the robot receives any movement command.
    if(fabs(follow_cmd.linear.x) > 0.01 || fabs(follow_cmd.angular.z) > 0.01){
        stop_count = 0;
        if(status != S_FOLLOW) {
            ROS_INFO("Target found, reverting to S_FOLLOW");
            status = S_FOLLOW;
            lostTriggered = false;
        }
    } else {
        stop_count++;
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

    // Publishers and Subscribers.
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/sensors/core", 10, &cliffCallback);

    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t timeReference = 0;
    bool backingSoundPlayed = false;
    ros::Time backupStartTime;
    ros::Time escapeStartTime;
    bool isCounting = false;
    ros::Duration debounceDuration(0.5);

    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    double angular = 0.2;
    double linear = 0.0;
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    ros::Duration(0.5).sleep();

    while(ros::ok() && secondsElapsed <= 480){		
        ros::spinOnce();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start).count();

        // LOST TRACKING CHECK:
        // Only trigger lost state if stop_count exceeds threshold AND lost hasn't been triggered.
        if(status == S_FOLLOW && stop_count > 5 && !lostTriggered) {
            lostTriggered = true;
            handleLostTrack();
            sc.playWave(path_to_sounds + "Sad Violin Sound Effect.wav");  // Use a gentle, sad sound.
            status = S_PLACEHOLDER;
        }

        switch(status){
            case S_FOLLOW:
            {
                ROS_INFO("STATE: S_FOLLOW");
                playingSound = false;
                vel_pub.publish(follow_cmd);
                break;
            }
            case S_BUMPER:
            {
                ROS_INFO("STATE: BUMPER PRESSED EVENT");
                // Cancel any escape mode.
                if(status == S_ESCAPE) {
                    status = S_FOLLOW;
                    playingSound = false;
                }
                
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Anger Management.wav");  // Use a deep, grumbling sound for anger.
                }

                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 2){
                    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now()-start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }
							
                // Vigorously shake left to right.
                // Increase number of iterations or adjust duration for extra effect.
                for (int i = 0; i < 5; i++){
                    ROS_INFO("Vigorously Shaking LEFT");
                    setMovement(vel, vel_pub, 0, 6.0);
                    ros::Duration(0.06).sleep();
                    ROS_INFO("Vigorously Shaking RIGHT");
                    setMovement(vel, vel_pub, 0, -6.0);
                    ros::Duration(0.06).sleep();
                }
				
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();				
                status = S_FOLLOW;  // Return to follow state.
                playingSound = false;
                break;
            }
            case S_ESCAPE:
            {
                ROS_WARN("STATE: ESCAPE MODE ACTIVATED!");
				
                if(!playingSound){
                    sc.playWave(path_to_sounds + "fear_scream.wav");
                    playingSound = true;
                }

                ros::Duration escapeDuration = ros::Time::now() - escapeStartTime;
				
                if(escapeDuration < ros::Duration(2.0)) {
                    // First phase: rapid rotation.
                    setMovement(vel, vel_pub, 0.0, 1.5);
                }
                else if(escapeDuration < ros::Duration(5.0)) {
                    // Second phase: rapid forward movement.
                    setMovement(vel, vel_pub, 0.5, 0.0);
                }
                else {
                    // End escape, resume following.
                    status = S_FOLLOW;
                    playingSound = false;
                    setMovement(vel, vel_pub, 0.0, 0.0);
                }
				
                break;
            }
            case S_CLIFF:
            {
                ROS_INFO("STATE: CLIFF ACTIVE EVENT");
                // For surprise (e.g., when the robot is picked up) use a high-pitched, sudden gasp.
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Suprised.wav");  // Ensure this file is a surprised sound effect.
                }
                setMovement(vel, vel_pub, 0, 0);
                break;
            }
            case S_PLACEHOLDER:
            {
                // In lost state, if movement is detected (target found), revert to follow.
                if(fabs(follow_cmd.linear.x) > 0.01 || fabs(follow_cmd.angular.z) > 0.01){
                    ROS_INFO("FOUND TARGET, reverting to S_FOLLOW");
                    status = S_FOLLOW;
                    lostTriggered = false;
                    playingSound = false;
                    break;
                }
                handleLostTrack();
                break;
            }
        }

        loop_rate.sleep();
    }

    return 0;
}

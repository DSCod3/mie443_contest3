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
bool lostTriggered = false;  // Prevent repeated lost event triggering
ros::Time lastFollowTime;    // Tracks time of the last follower command

// Follower callback: update command and reset lost-tracking state.
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    lastFollowTime = ros::Time::now();
    // If valid movement is detected, revert to follow state.
    if(fabs(follow_cmd.linear.x) > 0.01 || fabs(follow_cmd.angular.z) > 0.01){
        status = S_FOLLOW;
        lostTriggered = false;
    }
}

// Microphone callback: using pocketsphinx_ros output on "/recognizer/output".
// If the recognized text contains "good robot", trigger S_MICROPHONE.
void microphoneCB(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Speech recognizer output: %s", msg->data.c_str());
    // Optionally, add a confidence check if available.
    if(msg->data.find("good robot") != std::string::npos) {
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

    // Publishers and Subscribers.
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/events/cliff", 10, &cliffCallback);
    // Subscribe to the recognizer output (e.g., from pocketsphinx_ros).
    ros::Subscriber mic_sub = nh.subscribe("recognizer/output", 10, &microphoneCB);

    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
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
    lastFollowTime = ros::Time::now();

    while(ros::ok() && secondsElapsed <= 480){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start).count();

        // LOST TRACKING CHECK:
        // If in S_FOLLOW and no valid follower command for >3 seconds, switch to S_PLACEHOLDER.
        if(status == S_FOLLOW && (current_time - lastFollowTime).toSec() > 3.0 &&
           (fabs(follow_cmd.linear.x) < 0.01 && fabs(follow_cmd.angular.z) < 0.01)) {
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
                // Back up for 2 seconds.
                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 2){
                    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now() - start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }
                // Drastic shaking: alternate left/right with very high angular speed.
                for (int i = 0; i < 3; i++){
                    ROS_INFO("Shaking LEFT");
                    setMovement(vel, vel_pub, 0, 6.0); // High speed turning
                    ros::Duration(0.06).sleep();       // 3x faster than before (~0.06 sec)
                    ROS_INFO("Shaking RIGHT");
                    setMovement(vel, vel_pub, 0, -6.0);
                    ros::Duration(0.06).sleep();
                }
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();
                status = S_FOLLOW;  // Revert to follow state.
                playingSound = false;
                break;

            case S_CLIFF:
				ROS_INFO("CLIFF ACTIVE EVENT");

				setMovement(vel, vel_pub, 0, 0);
				
				// Cliff sensor needs to be debounced. 
				timeReference = secondsElapsed;
				while(secondsElapsed - timeReference < 1){
					if(status == S_CLIFF){
						secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
						timeReference = secondsElapsed;
					}
					ros::spinOnce();
				}
				
				if(!playingSound){
					playingSound = true;
					sc.playWave(path_to_sounds + "Girl Screaming Sound Effect.wav");
				}
				break;

            case S_MICROPHONE:
                ROS_INFO("STATE: MICROPHONE EVENT - INFATUATED (GOOD ROBOT)");
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Infatuated.wav");
                }
                setMovement(vel, vel_pub, 0, 0);
                status = S_FOLLOW;
                playingSound = false;
                break;

            case S_PLACEHOLDER:
                // Lost Tracking state: if a valid command is received, revert to follow.
                if(fabs(follow_cmd.linear.x) > 0.01 || fabs(follow_cmd.angular.z) > 0.01){
                    ROS_INFO("FOUND TARGET, reverting to S_FOLLOW");
                    status = S_FOLLOW;
                    lostTriggered = false;
                    playingSound = false;
                    break;
                }
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

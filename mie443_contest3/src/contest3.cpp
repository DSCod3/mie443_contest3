#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <bumper.h>
#include <cliff.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
int stop_count = 0;
Status status;
bool playingSound = false;

void setMovement(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, float lx, float rz){
    vel.angular.z = rz;
    vel.linear.x = lx;
    vel_pub.publish(vel);
    ros::spinOnce();
}

// Modified: Added parameters for minimal lost rotation functionality.
void handleLostTrack(ros::Publisher &vel_pub, geometry_msgs::Twist &vel) {
    ROS_INFO("Robot lost track. Rotating in place to search for target.");
    sound_play::SoundClient sc;
    string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    sc.playWave(path_to_sounds + "Sad_SPBB.wav");
    auto rotationStart = std::chrono::system_clock::now();
    while(ros::ok() && std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now() - rotationStart).count() < 3) {
        // Rotate slowly in place.
        setMovement(vel, vel_pub, 0, 0.3);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
    ROS_INFO("x, y, z: [%f, %f, %f]", msg.linear.x, msg.linear.y, msg.angular.z);
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
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/sensors/core", 10, &cliffCallback);

    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    uint64_t timeReference = 0;
    bool backingSoundPlayed = false;
    ros::Time backupStartTime;
    ros::Time escapeStartTime;
    bool isCounting = false;
    ros::Duration debounceDuration(0.5);

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

        switch(status){
            case S_FOLLOW:
            {
                ROS_INFO("S_FOLLOW");
                if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0) {
                    stop_count++;
                } else {
                    stop_count = 0; // Reset if the robot moves again
                }
                ROS_INFO("STOP COUNT: %d", stop_count);
			
                if (stop_count > 5) { // Threshold for lost tracking.
                    handleLostTrack(vel_pub, vel);  // Minimal modification: now rotates in place.
                    stop_count = 0; // Reset count after handling
                    sc.playWave(path_to_sounds + "Sad_SPBB.wav");
                    status = S_PLACEHOLDER;
                }

                playingSound = false;
                vel_pub.publish(follow_cmd);

                static bool last_back_state = false;
                bool current_back_state = (follow_cmd.linear.x < -0.01);
				
                if(current_back_state != last_back_state) {
                    backupStartTime = ros::Time::now();
                }
				
                if(current_back_state) {
                    if((ros::Time::now() - backupStartTime) > debounceDuration) {
                        if(!isCounting) {
                            backupStartTime = ros::Time::now();
                            isCounting = true;
                            ROS_INFO("Real backing started");
                        }
						
                        if((ros::Time::now() - backupStartTime).toSec() > 3.0 && !backingSoundPlayed) {
                            sc.playWave(path_to_sounds + "fear_scream.wav");
                            backingSoundPlayed = true;
                            ROS_WARN("Fear sound triggered after 3s");
                            status = S_ESCAPE;
                            escapeStartTime = ros::Time::now();
                        }
                    }
                } else {
                    isCounting = false;
                    if(backingSoundPlayed) {
                        backupStartTime = ros::Time::now() - ros::Duration(5.0); 
                    }
                    backingSoundPlayed = false;
                }
				
                last_back_state = current_back_state;
                break;
            }
            case S_BUMPER:{
                ROS_INFO("BUMPER PRESSED EVENT");
                if(status == S_ESCAPE) {
                    status = S_FOLLOW;
                    playingSound = false;
                }
				
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Angry.wav");
                }

                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 2){
                    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now()-start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }
							
                // Minimal modification: add shaking behavior.
                for (int i = 0; i < 5; i++){
                    ROS_INFO("Shaking LEFT");
                    setMovement(vel, vel_pub, 0, 6.0);
                    ros::Duration(0.06).sleep();
                    ROS_INFO("Shaking RIGHT");
                    setMovement(vel, vel_pub, 0, -6.0);
                    ros::Duration(0.06).sleep();
                }
				
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();				
                break;
            }
            case S_ESCAPE:
            {
                ROS_WARN("ESCAPE MODE ACTIVATED!");
                if(!playingSound){
                    sc.playWave(path_to_sounds + "fear_scream.wav");
                    playingSound = true;
                }
				
                ros::Duration escapeDuration = ros::Time::now() - escapeStartTime;
				
                if(escapeDuration < ros::Duration(2.0)) {
                    setMovement(vel, vel_pub, 0.0, 1.5);
                }
                else if(escapeDuration < ros::Duration(5.0)) {
                    setMovement(vel, vel_pub, 0.5, 0.0);
                }
                else {
                    status = S_FOLLOW;
                    playingSound = false;
                    setMovement(vel, vel_pub, 0.0, 0.0);
                }
				
                break;
            }
            case S_CLIFF:{
                ROS_INFO("CLIFF ACTIVE EVENT");
                if(status == S_ESCAPE) {
                    status = S_FOLLOW;
                    playingSound = false;
                }
                setMovement(vel, vel_pub, 0, 0);
                if(!playingSound){
                    playingSound = true;
                    sc.playWave(path_to_sounds + "Suprised.wav");
                }
                break;
            }
            case S_PLACEHOLDER:{
                handleLostTrack(vel_pub, vel);
                break;
            }
        }

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}

#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <bumper.h>
#include <cliff.h>
#include <fear.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
Status status;
bool playingSound = false;  // 已有但需要确认

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
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

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber cliff_sub = nh.subscribe("/mobile_base/sensors/core", 10, &cliffCallback);
	ros::Subscriber fear_sub = nh.subscribe("camera/image", 1, &fearCheckCallback);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
	uint64_t timeReference = 0;
	bool backingSoundPlayed = false; // Add this with other global variables

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	//sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		switch(status){

			case S_FEAR:
				handleFearState(vel, vel_pub, sc, path_to_sounds);
				break;

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

				timeReference = secondsElapsed;
				while(secondsElapsed - timeReference < 2){
					secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
					setMovement(vel, vel_pub, -0.2, 0);
				}
							
				setMovement(vel, vel_pub, 0, 0);
				ros::spinOnce();				
				
				break;

			case S_CLIFF:
				ROS_INFO("CLIFF ACTIVE EVENT");

				setMovement(vel, vel_pub, 0, 0);

				if(!playingSound){
					playingSound = true;
					sc.playWave(path_to_sounds + "Discontent.wav");
				}
				
				// Cliff sensor needs to be debounced. 
				break;

			case S_MICROPHONE:
				setMovement(vel, vel_pub, 0, 0);
				break;
			case S_PLACEHOLDER:
				setMovement(vel, vel_pub, 0, 0);
				break;
		}

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}

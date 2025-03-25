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

    // sc.playWave(path_to_sounds + "sound.wav");
    ros::Duration(0.5).sleep();

    while(ros::ok() && secondsElapsed <= 480){
        ros::spinOnce();

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
                        std::chrono::system_clock::now()-start).count();
                    setMovement(vel, vel_pub, -0.2, 0);
                }

                // After backing up, perform a shaking behavior.
                // Shake left-right for a few iterations.
                for (int i = 0; i < 3; i++){
                    // Shake left: turn right (positive angular velocity)
                    timeReference = secondsElapsed;
                    while(secondsElapsed - timeReference < 1){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now()-start).count();
                        setMovement(vel, vel_pub, 0, 0.5);
                    }
                    // Shake right: turn left (negative angular velocity)
                    timeReference = secondsElapsed;
                    while(secondsElapsed - timeReference < 1){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now()-start).count();
                        setMovement(vel, vel_pub, 0, -0.5);
                    }
                }

                // Stop movement after the bumper reaction.
                setMovement(vel, vel_pub, 0, 0);
                ros::spinOnce();
                break;

            case S_CLIFF:
                ROS_INFO("CLIFF ACTIVE EVENT");
                setMovement(vel, vel_pub, 0, 0);

                // Cliff sensor needs to be debounced.
                timeReference = secondsElapsed;
                while(secondsElapsed - timeReference < 1){
                    if(status == S_CLIFF){
                        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::system_clock::now()-start).count();
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
                setMovement(vel, vel_pub, 0, 0);
                break;
        }

		// if(cliffActive){
			
		// }

		// if(bumpers.anyPressed){
			
		// }


		// if(world_state == 0){
		// 	//fill with your code
		// 	//vel_pub.publish(vel);
		// 	vel_pub.publish(follow_cmd);

		// }else if(world_state == 1){
		// 	/*
		// 	...
		// 	...
		// 	*/
		// }

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
	
	return 0;
}

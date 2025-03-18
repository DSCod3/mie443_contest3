#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

// Global variables for tracking commands and stimulus states.
geometry_msgs::Twist follow_cmd;
ros::Time last_follow_time;    // Last time a follower command was received.
ros::Time last_emotion_time;   // Last time an emotion was played.
bool obstacleTriggered = false;
int world_state;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;
    // Update the time when a new follow command is received.
    last_follow_time = ros::Time::now();
}

void bumperCB(const geometry_msgs::Twist msg) {
    // For simplicity, any bumper callback event is taken as an obstacle stimulus.
    ROS_INFO("Bumper event detected: Obstacle encountered.");
    obstacleTriggered = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;
    // Get the full path to the sounds directory.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    
    // Create teleController instance (if needed for manual overrides; not used further here).
    teleController eStop;

    // Set up publishers and subscribers.
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // Initialize timing variables.
    ros::Time start = ros::Time::now();
    last_follow_time = ros::Time::now();
    last_emotion_time = ros::Time::now();
    uint64_t secondsElapsed = 0;

    // Set up image transporters for RGB and depth images.
    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    world_state = 0; // The default state for following.

    // Initialize a default velocity command.
    geometry_msgs::Twist vel;
    vel.angular.z = 0.2;
    vel.linear.x = 0.0;

    // Play a startup sound.
    sc.playWave(path_to_sounds + "sound.wav");
    ros::Duration(0.5).sleep();

    ros::Rate loop_rate(10);
    while(ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();

        if(world_state == 0) {
            // Publish follower command (the person-tracking command).
            vel_pub.publish(follow_cmd);
        }
        else if(world_state == 1) {
            // Additional states could be used to implement other behaviors.
        }

        // Stimulus 1: Lost Tracking
        // If no follow command is received for more than 3 seconds, trigger "lost" emotion.
        if ((current_time - last_follow_time).toSec() > 3.0 &&
            (current_time - last_emotion_time).toSec() > 3.0) {
            ROS_INFO("Lost track of user: triggering 'lost' emotion");
            sc.playWave(path_to_sounds + "lost.wav");
            last_emotion_time = current_time;
        }

        // Stimulus 2: Obstacle Detected
        if (obstacleTriggered && (current_time - last_emotion_time).toSec() > 3.0) {
            ROS_INFO("Obstacle encountered: triggering 'obstacle' emotion");
            sc.playWave(path_to_sounds + "obstacle.wav");
            obstacleTriggered = false;  // Reset flag after handling.
            last_emotion_time = current_time;
        }

        // Stimulus 3: Following Well (Additional stimulus)
        // If the robot has been receiving follow commands (i.e. moving) for more than 10 seconds,
        // trigger a "joy" emotion.
        if ((current_time - last_emotion_time).toSec() > 10.0 &&
            fabs(follow_cmd.linear.x) > 0.1) {
            ROS_INFO("Following well: triggering 'joy' emotion");
            sc.playWave(path_to_sounds + "joy.wav");
            last_emotion_time = current_time;
        }

        secondsElapsed = (current_time - start).toSec();
        loop_rate.sleep();
    }

    return 0;
}

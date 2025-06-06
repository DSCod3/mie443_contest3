#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <bumper.h>
#include <cliff.h>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;



geometry_msgs::Twist follow_cmd;
int world_state;
int stop_count = 0;
Status status;
bool playingSound = false;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;
    ROS_INFO("x, y, z: [%f, %f, %f]", msg.linear.x, msg.linear.y, msg.angular.z);

    // Evaluate the tracking condition
    if (msg.linear.x == 0 && msg.angular.z == 0) {
        if (++stop_count > 5) { // Confirm consistent lost track condition
            if (status != S_LOST_TRACK) { // Prevent re-triggering if already in lost track state
                status = S_LOST_TRACK;
            }
            stop_count = 0; // Optional: Reset stop_count if desired
        }
    } else {
        stop_count = 0; // Reset stop count whenever there's movement
    }
}



void setMovement(geometry_msgs::Twist &vel, ros::Publisher &vel_pub, float lx, float rz){
	vel.angular.z = rz;
	vel.linear.x = lx;
	vel_pub.publish(vel);
	ros::spinOnce();
}


void showEmojiFullscreen(const std::string& imagePath, int durationMs) {
	string path_to_emoji = ros::package::getPath("mie443_contest3") + "/emoji/";
    cv::Mat img = cv::imread(path_to_emoji + imagePath);
    if (img.empty()) return;

    cv::namedWindow("Emoji", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Emoji", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Emoji", img);
    cv::waitKey(1000);
    
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


    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
	uint64_t timeReference = 0;
	bool backingSoundPlayed = false; // Add this with other global variables
	ros::Time backupStartTime;
	ros::Time escapeStartTime;
	ros::Time lostTrackStartTime;
	bool isCounting = false;
	ros::Duration debounceDuration(0.5); // 防抖动时间窗口

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
			case S_FOLLOW:
			{
				ROS_INFO("S_FOLLOW");

				cv::destroyAllWindows();
				////////handle lost
				if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0) {
					stop_count++;
				} else {
					stop_count = 0; // Reset if the robot moves again
				}
				ROS_INFO("STOP COUNT: %d", stop_count);
			
				if (stop_count > 5) { // Adjust threshold based on responsiveness needs
					status = S_LOST_TRACK;
					stop_count = 0; // Reset count after handling
				}


				playingSound = false;
			
				vel_pub.publish(follow_cmd);
			
				// 将相关变量声明移到case代码块内部，并用{}包裹整个case
				static bool last_back_state = false;
				bool current_back_state = (follow_cmd.linear.x < -0.01);
				
				// 状态变化检测（正向或负向边沿）
				if(current_back_state != last_back_state) {
					backupStartTime = ros::Time::now(); // 重置计时器当状态变化
				}
				
				if(current_back_state) {
					// 持续检测时间（包含防抖动）
					if((ros::Time::now() - backupStartTime) > debounceDuration) {
						// 有效持续后退开始计时
						if(!isCounting) {
							backupStartTime = ros::Time::now();
							isCounting = true;
							ROS_INFO("Real backing started");
						}
						
						// 主计时检测（3秒触发）
						if((ros::Time::now() - backupStartTime).toSec() > 3.0 && !backingSoundPlayed) {
							sc.playWave(path_to_sounds + "fear_scream.wav");
							backingSoundPlayed = true;
							ROS_WARN("Fear sound triggered after 3s");
							status = S_ESCAPE;  // 满足条件进入逃跑状态
							escapeStartTime = ros::Time::now();
						}
					}
				} else {
					// 重置所有状态
					isCounting = false;
					if(backingSoundPlayed) {
						// 添加冷却时间（5秒内不再触发）
						backupStartTime = ros::Time::now() - ros::Duration(5.0); 
					}
					backingSoundPlayed = false;
				}
				
				last_back_state = current_back_state;
				break;
			}
			case S_BUMPER:{
				ROS_INFO("BUMPER PRESSED EVENT");

				// 遇到碰撞时取消逃跑状态
				if(status == S_ESCAPE) {
					status = S_FOLLOW;
					playingSound = false;
				}
				
				if(!playingSound){
					playingSound = true;
					sc.playWave(path_to_sounds + "Angry.wav");
					showEmojiFullscreen("very_angry_emoji.png", 2000);
				}

				timeReference = secondsElapsed;
				while(secondsElapsed - timeReference < 3){
					secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
					setMovement(vel, vel_pub, -0.2, 0);
				}
							
				setMovement(vel, vel_pub, 0, 0);
				ros::spinOnce();				
				
				break;
			}

			case S_ESCAPE:
			{
				ROS_WARN("ESCAPE MODE ACTIVATED!");
				
				// 1. 播放恐惧声音
				if(!playingSound){
					sc.playWave(path_to_sounds + "fear_scream.wav");
					showEmojiFullscreen("fear_emoji.png", 2000);
					playingSound = true;
				}

				// 2. 执行逃跑动作（先旋转180度，然后前进）
				ros::Duration escapeDuration = ros::Time::now() - escapeStartTime;
				
				if(escapeDuration < ros::Duration(2.0)) {
					// 第一阶段：快速旋转
					setMovement(vel, vel_pub, -0.3, 0);  // 快速后退， 看看会不会丢目标
				}
				else if(escapeDuration < ros::Duration(2.5)) {
					// 第二阶段：直线逃跑
					setMovement(vel, vel_pub, 0,-2);  // 快速前进
				}
				else if(escapeDuration < ros::Duration(3)) {
					// 第二阶段：直线逃跑
					setMovement(vel, vel_pub, 0, 2);  // 快速前进
				}
				else {
					// 逃跑结束恢复跟随
					status = S_FOLLOW;
					playingSound = false;
					setMovement(vel, vel_pub, 0.0, 0.0);  // 停止运动
				}
				
				break;
			}

			case S_CLIFF:{
				ROS_INFO("CLIFF ACTIVE EVENT");

				// 遇到碰撞时取消逃跑状态
				if(status == S_ESCAPE) {
					status = S_FOLLOW;
					playingSound = false;
				}

				setMovement(vel, vel_pub, 0, 0);

				if(!playingSound){
					playingSound = true;
					sc.playWave(path_to_sounds + "Proud.wav");
					showEmojiFullscreen("proud_emoji.png", 2000); // Proud
				}
				
				// Cliff sensor needs to be debounced. 
				break;
			}

			case S_LOST_TRACK: {
				ROS_INFO("Robot lost track.");
				if (!playingSound) {
					sc.playWave(path_to_sounds + "Sad_SPBB.wav");
    				showEmojiFullscreen("sad_emoji.png", 2000); // Sad
					playingSound = true; // Set playing to true to avoid replaying sound
				}
			
				// This checks if the robot continues to be in lost track state
				if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0) {
					// Still lost, do nothing more, just wait
				} else {
					// If we have movement again, assume we are tracking again
					playingSound = false; // Reset playing state
					status = S_FOLLOW; // Change status back to follow
				}
				break;
			}
			

		}

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}

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
bool playingSound = false;



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


    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
	uint64_t timeReference = 0;
	bool backingSoundPlayed = false; // Add this with other global variables
	ros::Time backupStartTime;
	ros::Time escapeStartTime;
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
			}

			case S_ESCAPE:
			{
				ROS_WARN("ESCAPE MODE ACTIVATED!");
				
				// 1. 播放恐惧声音
				if(!playingSound){
					sc.playWave(path_to_sounds + "fear_scream.wav");
					playingSound = true;
				}

				// 2. 执行逃跑动作（先旋转180度，然后前进）
				ros::Duration escapeDuration = ros::Time::now() - escapeStartTime;
				
				if(escapeDuration < ros::Duration(2.0)) {
					// 第一阶段：快速旋转
					setMovement(vel, vel_pub, 0.0, 1.5);  // 原地旋转
				}
				else if(escapeDuration < ros::Duration(5.0)) {
					// 第二阶段：直线逃跑
					setMovement(vel, vel_pub, 0.5, 0.0);  // 快速前进
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
					sc.playWave(path_to_sounds + "Discontent.wav");
				}
				
				// Cliff sensor needs to be debounced. 
				break;
			}

			case S_PLACEHOLDER:{
				setMovement(vel, vel_pub, 0, 0);
				break;
			}
		}

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ArmVel.h>

// ロボットの自己位置推定信号からアームの開閉判断信号に変換
class ArmQueueConverter{
	private:
	float theta_now = M_PI/2; // オドメトリ座標系から見た現在のロボットの角度(rad)

	public:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Publisher _arm_pub;
	ros::Subscriber _vel_sub;
	ros::Timer _timer;
	custom_msgs::ArmVel _last_vel;

	// コンストラクタ
	ArmQueueConverter() : _nh(), _pnh("~"){
		_arm_pub = _nh.advertise<custom_msgs::ArmVel>("arm_vel",1);
		_vel_sub = _nh.subscribe("cmd_armvel", 10, &ArmQueueConverter::velCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &ArmQueueConverter::timerCb, this);
	}

	void velCb(const custom_msgs::ArmVel &cmd_vel_msg){
		_last_vel = cmd_vel_msg;
	}
 
	void timerCb(const ros::TimerEvent& e){
		
		// 以下をターミナルに打ち込んで表示
		/* rosrun rqt_console rqt_console */
		ROS_INFO("target_arm_vel:%f", _last_vel.vel);
		
		// 角速度を[rad/s]からエンコーダの[count/s]に変換
		custom_msgs::ArmVel arm_vel;
		arm_vel.vel = _last_vel.vel;
		arm_vel.armopen = true;
		// if(アームを広げたい条件){
		// 	arm_vel.armopen = true;
		// }else if(アームを閉じたい条件){
		// 	arm_vel.armopen = false;
		// }

		_arm_pub.publish(arm_vel);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_wheel_drive");
  ArmQueueConverter speed_converter;
  ros::Rate rate(100);
  while(ros::ok()){
  	ros::spinOnce();
	rate.sleep();
  }
  return 0;
}
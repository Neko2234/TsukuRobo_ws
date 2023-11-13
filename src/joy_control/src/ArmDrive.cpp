#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ArmVel.h>

// ロボット速度指令をタイヤへの回転速度指令に変換
class TwoWDSpeedConverter{
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
	TwoWDSpeedConverter() : _nh(), _pnh("~"){
		_arm_pub = _nh.advertise<custom_msgs::ArmVel>("cmd_arm",1);
		_vel_sub = _nh.subscribe("cmd_armvel", 10, &TwoWDSpeedConverter::velCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &TwoWDSpeedConverter::timerCb, this);
	}

	void velCb(const custom_msgs::ArmVel &cmd_vel_msg){
		_last_vel = cmd_vel_msg;
	}
 
	void timerCb(const ros::TimerEvent& e){
		
		// 以下をターミナルに打ち込んで表示
		/* rosrun rqt_console rqt_console */
		ROS_INFO("target_arm_vel:%f", _last_vel.vel);
		
		// 角速度を[rad/s]からエンコーダの[count/s]に変換
		custom_msgs::ArmVel cmd_arm;
		cmd_arm.vel = _last_vel.vel;

		_arm_pub.publish(cmd_arm);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_wheel_drive");
  TwoWDSpeedConverter speed_converter;
  ros::Rate rate(100);
  while(ros::ok()){
  	ros::spinOnce();
	rate.sleep();
  }
  return 0;
}
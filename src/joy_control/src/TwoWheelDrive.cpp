#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/TwoWDPower.h>

// ロボット速度指令をタイヤへの回転速度指令に変換
class SpeedConverter{
	private:
	float theta = 0; // 最初の向きからどれだけの角度動いたか(rad)

	public:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Publisher _w_pub;
	ros::Subscriber _vel_sub;
	ros::Timer _timer;
	geometry_msgs::Twist _last_vel;

	SpeedConverter() : _nh(), _pnh("~"){
		_w_pub = _nh.advertise<geometry_msgs::Twist>("arduino/cmd_w",1);
		_vel_sub = _nh.subscribe("twoWD/cmd_vel", 10, &SpeedConverter::wCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &SpeedConverter::timerCb, this);
	}

	void wCb(const geometry_msgs::Twist& cmd_vel_msg){
		_last_vel = cmd_vel_msg;
	}

	void timerCb(const ros::TimerEvent& e){
		float first_x = 0.; // (m)
		float first_y = 0.; // (m)
		float first_z = M_PI/2; // 初期位置ではロボットは正面を向く
		float tires_dist = 0.3; // タイヤ間の距離(m)
		float radius = 0.1; // タイヤの半径(m)
		_pnh.getParam("first_x", first_x);
		_pnh.getParam("first_y", first_y);
		_pnh.getParam("first_z", first_z);
		_pnh.getParam("tires_dist", tires_dist);

		// 両輪独立駆動ロボットの逆運動学計算
		custom_msgs::TwoWDPower cmd_w;
		cmd_w.L = _last_vel.linear.x / (radius * cos(theta + first_z)) - (_last_vel.angular.z * tires_dist) / (2*radius);
		cmd_w.R = _last_vel.linear.y / (radius * sin(theta + first_z)) + (_last_vel.angular.z * tires_dist) / (2*radius);

		// 両輪独立駆動ロボットの運動学計算


		_w_pub.publish(cmd_w);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_wheel_drive");
  SpeedConverter speed_converter;
  ros::spin();
  return 0;
}
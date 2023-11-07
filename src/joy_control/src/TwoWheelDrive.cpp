#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/TwoWDPower.h>

// ロボット速度指令をタイヤへの回転速度指令に変換
class TwoWDSpeedConverter{
	private:
	float theta = 0; // 最初の向きからどれだけの角度動いたか(rad)

	public:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Publisher _w_pub;
	ros::Subscriber _vel_sub;
	ros::Timer _timer;
	geometry_msgs::Twist _last_vel;

	// コンストラクタ
	TwoWDSpeedConverter() : _nh(), _pnh("~"){
		_w_pub = _nh.advertise<geometry_msgs::Twist>("arduino/cmd_w",1);
		_vel_sub = _nh.subscribe("twoWD/cmd_vel", 10, &TwoWDSpeedConverter::wCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &TwoWDSpeedConverter::timerCb, this);
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
		int encoder_cpr = 2048 * 4; // エンコーダのcounts per revolution [count/rev]
		_pnh.getParam("first_x", first_x);
		_pnh.getParam("first_y", first_y);
		_pnh.getParam("first_z", first_z);
		_pnh.getParam("tires_dist", tires_dist);
		_pnh.getParam("encoder_cpr", encoder_cpr);

		// 両輪独立駆動ロボットの逆運動学計算
		float ang_vel[2]; // 角速度[rad/s], L,Rの順
		ang_vel[0] = _last_vel.linear.x / (radius * cos(theta + first_z)) - (_last_vel.angular.z * tires_dist) / (2*radius);
		ang_vel[1] = _last_vel.linear.y / (radius * sin(theta + first_z)) + (_last_vel.angular.z * tires_dist) / (2*radius);

		// 角速度を[rad/s]からエンコーダの[count/s]に変換
		custom_msgs::TwoWDPower cmd_w;
		cmd_w.L = ang_vel[0]/(2*M_PI) * encoder_cpr;
		cmd_w.R = ang_vel[1]/(2*M_PI) * encoder_cpr;

		_w_pub.publish(cmd_w);
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "two_wheel_drive");
  TwoWDSpeedConverter speed_converter;
  ros::spin();
  return 0;
}
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/TwoWDAngVel.h>
#include <std_msgs/Bool.h>

// ロボット速度指令をタイヤへの回転速度指令に変換
class TwoWDSpeedConverter
{
private:
	bool safe_signal = false;

public:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Publisher _w_pub;
	ros::Subscriber _vel_sub;
	ros::Subscriber _safe_signal_sub;
	ros::Timer _timer;
	geometry_msgs::Twist _last_vel;

	// コンストラクタ
	TwoWDSpeedConverter() : _nh(), _pnh("~")
	{
		_w_pub = _nh.advertise<custom_msgs::TwoWDAngVel>("cmd_w", 1);
		_vel_sub = _nh.subscribe("twoWD/cmd_vel", 10, &TwoWDSpeedConverter::velCb, this);
		_safe_signal_sub = _nh.subscribe("twoWD/safe_signal", 10, &TwoWDSpeedConverter::safeSignalCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &TwoWDSpeedConverter::timerCb, this);
	}

	void velCb(const geometry_msgs::Twist &cmd_vel_msg)
	{
		_last_vel = cmd_vel_msg;
	}

	void safeSignalCb(const std_msgs::Bool &safe_signal_msg)
	{
		safe_signal = safe_signal_msg.data;
	}

	void timerCb(const ros::TimerEvent &e)
	{
		float first_x = 0.;			// (m)
		float first_y = 0.;			// (m)
		float tread = 0.3;			// タイヤ間の距離(m)
		float radius = 0.1;			// タイヤの半径(m)
		int encoder_cpr = 2048 * 4; // エンコーダのcounts per revolution [count/rev]
		_pnh.getParam("/twoWD/first_x", first_x);
		_pnh.getParam("/twoWD/first_y", first_y);
		_pnh.getParam("/twoWD/tread", tread);
		_pnh.getParam("/twoWD/radius", radius);
		_pnh.getParam("/twoWD/encoder_cpr", encoder_cpr);

		custom_msgs::TwoWDAngVel cmd_w; // タイヤの角速度指令[count/s]
		if (safe_signal == false)
		{
			ROS_DEBUG("safe_signal is false");
			cmd_w.L = 0;
			cmd_w.R = 0;
		}
		else
		{
			// 指示された速度からロボット座標系から見た進む角度と速さを求める
			float cmd_ang = atan2(_last_vel.linear.x, _last_vel.angular.z);
			float cmd_v = hypot(_last_vel.angular.z, _last_vel.linear.x); // 平方和の平方根(直角三角形の斜辺)

			// 両輪独立駆動ロボットの逆運動学計算
			float wheel_vel[2]; // タイヤの角速度[rad/s], L,Rの順
			wheel_vel[0] = cmd_v * sin(cmd_ang - M_PI / 4);
			wheel_vel[1] = cmd_v * cos(cmd_ang - M_PI / 4);

			// 以下をターミナルに打ち込んで表示
			/* rosrun rqt_console rqt_console */
			ROS_DEBUG("target_ang_velL:%f", wheel_vel[0]);
			ROS_DEBUG("target_ang_velR:%f", wheel_vel[1]);

			// 角速度を[rad/s]からエンコーダの[count/s]に変換
			cmd_w.L = wheel_vel[0] / (2 * M_PI) * encoder_cpr;
			cmd_w.R = wheel_vel[1] / (2 * M_PI) * encoder_cpr;
		}

		_w_pub.publish(cmd_w);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "two_wheel_drive");
	TwoWDSpeedConverter speed_converter;
	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
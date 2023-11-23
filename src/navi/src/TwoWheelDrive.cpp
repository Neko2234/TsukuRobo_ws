#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/TwoWDAngVel.h>

// ロボット速度指令をタイヤへの回転速度指令に変換
class TwoWDSpeedConverter
{
	// private:
	// 	float theta_now = M_PI / 2; // オドメトリ座標系から見た現在のロボットの角度(rad)

public:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Publisher _w_pub;
	ros::Subscriber _vel_sub;
	ros::Timer _timer;
	geometry_msgs::Twist _last_vel;

	// コンストラクタ
	TwoWDSpeedConverter() : _nh(), _pnh("~")
	{
		_w_pub = _nh.advertise<custom_msgs::TwoWDAngVel>("cmd_w", 1);
		_vel_sub = _nh.subscribe("twoWD/cmd_vel", 10, &TwoWDSpeedConverter::velCb, this);
		// 10Hzでタイマーコールバックを呼び出す。
		_timer = _nh.createTimer(ros::Duration(0.1), &TwoWDSpeedConverter::timerCb, this);
	}

	void velCb(const geometry_msgs::Twist &cmd_vel_msg)
	{
		_last_vel = cmd_vel_msg;
	}

	void timerCb(const ros::TimerEvent &e)
	{
		float first_x = 0.;			// (m)
		float first_y = 0.;			// (m)
		float tread = 0.3;		// タイヤ間の距離(m)
		float radius = 0.1;			// タイヤの半径(m)
		int encoder_cpr = 2048 * 4; // エンコーダのcounts per revolution [count/rev]
		_pnh.getParam("/twoWD/first_x", first_x);
		_pnh.getParam("/twoWD/first_y", first_y);
		_pnh.getParam("/twoWD/tread", tread);
		_pnh.getParam("/twoWD/radius", radius);
		_pnh.getParam("/twoWD/encoder_cpr", encoder_cpr);

		// 指示された速度からロボット座標系から見た進む角度と速さを求める
		float cmd_ang = atan2(_last_vel.linear.y, _last_vel.linear.x);
		float cmd_v = hypot(_last_vel.linear.x, _last_vel.linear.y);

		// 両輪独立駆動ロボットの逆運動学計算
		float ang_vel[2]; // タイヤの角速度[rad/s], L,Rの順
		ang_vel[0] = cmd_v * cos(cmd_ang - M_PI / 4);
		ang_vel[1] = cmd_v * sin(cmd_ang - M_PI / 4);

		// 以下をターミナルに打ち込んで表示
		/* rosrun rqt_console rqt_console */
		ROS_DEBUG("target_ang_velL:%f", ang_vel[0]);
		ROS_DEBUG("target_ang_velR:%f", ang_vel[1]);

		// 角速度を[rad/s]からエンコーダの[count/s]に変換
		custom_msgs::TwoWDAngVel cmd_w;
		cmd_w.L = ang_vel[0] / (2 * M_PI) * encoder_cpr;
		cmd_w.R = ang_vel[1] / (2 * M_PI) * encoder_cpr;

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
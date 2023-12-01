#include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class TwistPublisher
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	// ros::Publisher cmd_pub_;
	ros::Publisher safe_signal_pub_;
	ros::Subscriber joy_sub_;
	ros::Timer timer_;
	sensor_msgs::Joy last_joy_;
	// ジョイコンの割り当て(yamlファイルから設定)
	// int assign_x = 1;
	// int assign_y = 0;
	// int assign_z = 2;
	int safe_button = 7; // このボタンを押しながらじゃないと動かない
						 // // /* 十字ボタンで直進と回転
						 // int assign_x_straight = 7;
						 // int assign_z_rotate = 6;
						 // // */

public:
	TwistPublisher() : nh_(), pnh_("~")
	{
		// cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		safe_signal_pub_ = nh_.advertise<std_msgs::Bool>("safe_signal", 1);
		joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
		timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
		// pnh_.getParamCached("/joy/assign_x", assign_x);
		// pnh_.getParamCached("/joy/assign_y", assign_y);
		// pnh_.getParamCached("/joy/assign_z", assign_z);
		pnh_.getParamCached("/joy/safe_button", safe_button);
		// pnh_.getParamCached("/joy/assign_x_straight", assign_x_straight);
		// pnh_.getParamCached("/joy/assign_z_rotate", assign_z_rotate);
	}

	void joyCallback(const sensor_msgs::Joy &joy_msg)
	{
		last_joy_ = joy_msg;
	}

	void timerCallback(const ros::TimerEvent &e)
	{
		// 最大速度の設定
		// float max_x = 1.0;
		// float max_y = 1.0;
		// float max_z = 1.0;
		// pnh_.getParamCached("/joy/max_x", max_x);
		// pnh_.getParamCached("/joy/max_y", max_y);
		// pnh_.getParamCached("/joy/max_z", max_z);

		// geometry_msgs::Twist cmd_vel;

		/* joyの入力からcmd_velを計算 */
		// /* 十字ボタンで直進と回転
		// if (0 <= assign_x_straight && assign_x_straight < last_joy_.axes.size())
		// {
		// 	cmd_vel.linear.x = max_x * last_joy_.axes[assign_x_straight];
		// }
		// if (0 <= assign_z_rotate && assign_z_rotate < last_joy_.axes.size())
		// {
		// 	cmd_vel.angular.z = max_z * last_joy_.axes[assign_z_rotate];
		// }
		// // */
		// if (0 <= assign_x && assign_x < last_joy_.axes.size())
		// {
		// 	if (cmd_vel.linear.x == 0)
		// 		cmd_vel.linear.x = max_x * last_joy_.axes[assign_x];
		// }
		// if (0 <= assign_y && assign_y < last_joy_.axes.size())
		// {
		// 	cmd_vel.linear.y = max_y * last_joy_.axes[assign_y];
		// }
		// if (0 <= assign_z && assign_z < last_joy_.axes.size())
		// {
		// 	if (cmd_vel.angular.z == 0)
		// 		cmd_vel.angular.z = max_z * last_joy_.axes[assign_z];
		// }
		/* cmd_velをpublish */
		// cmd_pub_.publish(cmd_vel);
		// }

		/* 安全ボタンの処理 */
		std_msgs::Bool safe_signal;
		if (0 <= safe_button && safe_button < last_joy_.buttons.size())
		{
			safe_signal.data = last_joy_.buttons[safe_button];
		}
		safe_signal_pub_.publish(safe_signal);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_twist_publisher");
	TwistPublisher twist_publisher;

	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

// LiDARのscanデータから壁からの距離を取得、その距離が一定値を超えるまで直進する
// 一定値を超えたら、次の状態に遷移する

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <custom_msgs/ArmVel.h>
#include <math.h>

enum class WallFollowState
{
	GOING,
	PUTTING,
	BACKING,

};

class WallFollow
{
private:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Subscriber _scan_sub;
	ros::Publisher _twist_pub;
	ros::Publisher _arm_vel_pub;
	ros::Timer _timer;
	WallFollowState _state;
	custom_msgs::ArmVel _arm_vel;
	int _range_size = 10; // 中心から左右に何個のデータを取るかを定義
	double _max_x = 0.01;
	double _max_z = 0.01;
	double goal_x = 4.5;
	double _average_distance_left;
	double _average_distance_right;
	double _average_distance_center;

public:
	WallFollow() : _nh(), _pnh("~")
	{
		_scan_sub = _nh.subscribe("scan", 10, &WallFollow::scanCb, this);
		_twist_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		_arm_vel_pub = _nh.advertise<custom_msgs::ArmVel>("arm_vel", 1);
		_state = WallFollowState::GOING;
		_timer = _nh.createTimer(ros::Duration(0.1), &WallFollow::timerCallback, this);
		_pnh.getParam("/wall_follow/range_size", _range_size);
		_pnh.getParam("/joy/max_x", _max_x);
		_pnh.getParam("/joy/max_z", _max_z);
	}

	void scanCb(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		// 中心のインデックスを計算
		int center_index = msg->ranges.size() / 2;

		// 範囲が配列の範囲内に収まるように調整
		int start_index_center = std::max(0, center_index - _range_size);
		int end_index_center = std::min((int)msg->ranges.size() - 1, center_index + _range_size);

		int start_index_left = 0;
		int end_index_left = std::min((int)msg->ranges.size() - 1, _range_size);

		int start_index_right = std::max(0, (int)msg->ranges.size() - 1 - _range_size);
		int end_index_right = msg->ranges.size() - 1;

		// 選択した範囲のデータを平均化
		float sum_center = 0, sum_left = 0, sum_right = 0;
		int count_center = 0, count_left = 0, count_right = 0;

		for (int i = start_index_center; i <= end_index_center; ++i)
		{
			if (!std::isnan(msg->ranges[i]))
			{
				sum_center += msg->ranges[i];
				++count_center;
			}
		}

		for (int i = start_index_left; i <= end_index_left; ++i)
		{
			if (!std::isnan(msg->ranges[i]))
			{
				sum_left += msg->ranges[i];
				++count_left;
			}
		}

		for (int i = start_index_right; i <= end_index_right; ++i)
		{
			if (!std::isnan(msg->ranges[i]))
			{
				sum_right += msg->ranges[i];
				++count_right;
			}
		}

		if (count_center > 0)
		{
			_average_distance_center = sum_center / count_center;
			ROS_INFO("Average distance to the front wall: %f", _average_distance_center);
		}

		if (count_left > 0)
		{
			_average_distance_left = sum_left / count_left;
			ROS_INFO("Average distance to the left wall: %f", _average_distance_left);
		}

		if (count_right > 0)
		{
			_average_distance_right = sum_right / count_right;
			ROS_INFO("Average distance to the right wall: %f", _average_distance_right);
		}
	}

	// 現在の左右の壁の距離からx速度とyaw角速度を決定する
	void calcGoVel(geometry_msgs::Twist &twist)
	{
		_pnh.getParam("/wall_follow/goal_x", goal_x);
		twist.linear.x = _max_x;
		twist.angular.z = (_average_distance_left - _average_distance_right) * _max_z;
		ROS_INFO("average_distance_right: %f", _average_distance_right);
		ROS_INFO("average_distance_left: %f", _average_distance_left);
	};
	void calcBackVel(geometry_msgs::Twist &twist)
	{
		twist.linear.x = -_max_x;
		twist.angular.z = (_average_distance_left - _average_distance_right) * _max_z;
		ROS_INFO("average_distance_right: %f", _average_distance_right);
		ROS_INFO("average_distance_left: %f", _average_distance_left);
	};

	void timerCallback(const ros::TimerEvent &e)
	{
		geometry_msgs::Twist cmd_vel;

		if (_state == WallFollowState::GOING)
		{
			calcGoVel(cmd_vel);
			if (_average_distance_center > goal_x)
			{
				_state = WallFollowState::PUTTING;
			}
		}
		else if (_state == WallFollowState::PUTTING)
		{
			_arm_vel.armopen = 1;
			_arm_vel_pub.publish(_arm_vel);
			_state = WallFollowState::BACKING;
		}
		else if (_state == WallFollowState::BACKING)
		{
			calcBackVel(cmd_vel);
		}

		_twist_pub.publish(cmd_vel);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Wall");
	WallFollow wall_follow;

	ros::Rate rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
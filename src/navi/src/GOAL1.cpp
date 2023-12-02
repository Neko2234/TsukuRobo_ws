// ゴール1狙いの行動をする
// まっすぐ行ってゴールに近づいたらbool型をパブリッシュ、引き返して同じことをするクラス

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::doTransform()を使用するために必要
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

enum class NavState
{
	STANDBY,
	GETTING_GOAL,
	MOVING
};

class GOAL1
{
private:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener _tfListener;
	ros::Publisher _twist_pub;
	ros::Timer _timer;
	ros::Publisher _marker_pub;

	NavState _nav_state;
	geometry_msgs::PoseStamped _last_pose;
	geometry_msgs::PoseStamped _goal_pose;

	double _goal_tolerance = 0.1; // 目標地点の許容誤差
	double _max_vel = 0.1;
	double _min_vel = 0.01;
	double _max_ang_vel = 0.1;
	double _min_ang_vel = 0.01;
	double _kp = 0.5;
	double _ka = 0.5;
	double _kb = 0.0;

public:
	GOAL1() : _nh(), _pnh("~"), _tfBuffer(), _tfListener(_tfBuffer)
	{
		_twist_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		_nav_state = NavState::STANDBY;
		_timer = _nh.createTimer(ros::Duration(0.2), &GOAL1::timerCb, this);
		_pnh.param("/move_navigation/goal_tolerance", _goal_tolerance);
		_pnh.param("/move_navigation/max_vel", _max_vel);
		_pnh.param("/move_navigation/min_vel", _min_vel);
		_pnh.param("/move_navigation/max_ang_vel", _max_ang_vel);
		_pnh.param("/move_navigation/min_ang_vel", _min_ang_vel);
		_pnh.param("/move_navigation/kp", _kp);
		_pnh.param("/move_navigation/ka", _ka);
		_pnh.param("/move_navigation/kb", _kb);
		_marker_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	}

	bool get_tf(geometry_msgs::TransformStamped &tf, const std::string parent_frame, const std::string child_frame, const float timeout)
	{
		// 時刻を記録する
		ros::Time now = ros::Time::now();

		// すべてのTFが補間可能になるまで待つ
		if (!tfBuffer.canTransform(parent_frame, child_frame, now, ros::Duration(timeout)))
		{
			ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]", parent_frame.c_str(), child_frame.c_str(), timeout);
			return false;
		}

		// nowの時刻でのtransformを取得する
		try
		{
			tf = tfBuffer.lookupTransform(parent_frame, child_frame, now, ros::Duration(0.1));
			return true;
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
			return false;
		}
	}

	void timerCb(const ros::TimerEvent &e)
	{
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.angular.z = 0.0;
		ros::Time now = ros::Time::now();
		try
		{
			// 現在のロボット座標を取得
			geometry_msgs::TransformStamped tf;
			get_tf(tf, "map", "base_link", 0.1);
			geometry_msgs::PoseStamped robot_pose;

			// ロボットの位置と姿勢を取得
			listener.lookupTransform("/map", "/base_link", ros::Time(0), tf);
			x = tf.getOrigin().x();
			y = tf.getOrigin().y();
			yaw = tf::getYaw(tf.getRotation());
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
			continue;
		}

		// 現在のロボットの位置と姿勢を記録

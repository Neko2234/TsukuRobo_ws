// geometry_msgs::PoseStamped型の目標地点を受け取り経路設計およびTwist型のcmd_velを出力するプログラム

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::doTransform()を使用するために必要
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <navfn/navfn_ros.h>

enum class NavState
{
	STANDBY,
	WAIT_PLAN,
	MOVING
};

class MoveNavigation
{
private:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener _tfListener;
	ros::Subscriber _goal_sub;
	ros::Publisher _twist_pub;
	ros::Timer _timer;

	NavState _nav_state;
	geometry_msgs::PoseStamped _last_pose;
	std::vector<geometry_msgs::PoseStamped> _last_global_plan;

	costmap_2d::Costmap2DROS _global_costmap;
	costmap_2d::Costmap2DROS _local_costmap;
	navfn::NavfnROS _global_planner;
	base_local_planner::TrajectoryPlannerROS _local_planner;

	double _goal_tolerance = 0.1; // 目標地点の許容誤差
	double _max_vel = 0.1;
	double _min_vel = 0.01;
	double _max_ang_vel = 0.1;
	double _min_ang_vel = 0.01;
	double _kp = 0.5;
	double _ka = 0.5;
	double _kb = 0.0;

public:
	MoveNavigation() : _nh(), _pnh("~"), _tfBuffer(), _tfListener(_tfBuffer), _global_costmap("global_costmap", _tfBuffer), _local_costmap("local_costmap", _tfBuffer)
	{
		_goal_sub = _nh.subscribe("move_navigation/goal", 10, &MoveNavigation::goalCb, this);
		_twist_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		_global_planner.initialize("global_planner", &_global_costmap); // 第1引数はプランナーの名前、第2引数はコストマップへのポインタ
		_local_planner.initialize("local_planner", &_tfBuffer, &_local_costmap);
		_nav_state = NavState::STANDBY;
		_timer = _nh.createTimer(ros::Duration(0.2), &MoveNavigation::timerCb, this);
		_pnh.param("/move_navigation/goal_tolerance", _goal_tolerance);
		_pnh.param("/move_navigation/max_vel", _max_vel);
		_pnh.param("/move_navigation/min_vel", _min_vel);
		_pnh.param("/move_navigation/max_ang_vel", _max_ang_vel);
		_pnh.param("/move_navigation/min_ang_vel", _min_ang_vel);
		_pnh.param("/move_navigation/kp", _kp);
		_pnh.param("/move_navigation/ka", _ka);
		_pnh.param("/move_navigation/kb", _kb);
	}

	void goalCb(const geometry_msgs::PoseStamped &msg)
	{
		ROS_INFO("Goal received");
		_last_pose = msg;
		_nav_state = NavState::WAIT_PLAN;
	}

	void timerCb(const ros::TimerEvent &e)
	{
		/***************************************************************/
		/************************* 経路計画待ち *************************/
		/***************************************************************/
		if (_nav_state == NavState::WAIT_PLAN)
		{
			ROS_INFO("Planning...");

			// base_linkからmapへの姿勢変換を取得
			geometry_msgs::PoseStamped source_pose; // base_linkから見た目標地点の姿勢
			source_pose.header.frame_id = "base_link";
			source_pose.header.stamp = ros::Time::now();
			source_pose.pose.orientation.w = 1.0;

			geometry_msgs::PoseStamped target_pose; // mapからみたロボットの目標姿勢
			std::string target_frame = "map";
			// mapのtfが取得できる状態になるまで待機
			if (!_tfBuffer.canTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0))) // ros::Time(0) を指定した場合、source_frame と target_frame の共通で利用できる最新の時刻を探す
			{
				// tfが取得できなかった場合
				ROS_INFO("[MOVE_NAVIGATION ERROR]: Failed to get robot pose");
				return;
			}

			try
			{
				// base_linkからmapへの変換を取得
				geometry_msgs::TransformStamped transformStamped = _tfBuffer.lookupTransform(source_pose.header.frame_id, target_frame, ros::Time(0));

				// 取得した変換を使用して姿勢を変換(source_poseをtransformStampedで変換したものをtarget_frameに格納)
				tf2::doTransform(source_pose, target_pose, transformStamped);

				ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s", ex.what());
			}

			geometry_msgs::PoseStamped start_pose = target_pose;

			// グローバルプランナーによる経路計画
			if (!_global_planner.makePlan(start_pose, _last_pose, _last_global_plan)) // スタート位置（start_pose）からゴール位置（_last_pose）までの最適な経路を計算し、その結果を_last_global_planに格納
			{
				// 経路が見つからなかった場合
				ROS_WARN("[MOVE_NAVIGATION ERROR]: Failed to make a global plan");
				_nav_state = NavState::STANDBY;
				return;
			}
			ROS_INFO("Global plan is made");

			_local_planner.setPlan(_last_global_plan);
			geometry_msgs::Twist cmd_vel;
			if (_local_planner.isGoalReached())
			{
				// 目標地点に到達した場合
				ROS_INFO("reach");
				_twist_pub.publish(cmd_vel);
				_nav_state = NavState::STANDBY;
				return;
			}
			_twist_pub.publish(cmd_vel);
			_nav_state = NavState::MOVING;
		}
		/**********************************************************/
		/************************* 移動中 *************************/
		/**********************************************************/
		else if (_nav_state == NavState::MOVING)
		{
			ROS_INFO_THROTTLE(2.0, "Moving..."); // 2秒に1回表示
			geometry_msgs::Twist cmd_vel;
			if (_local_planner.isGoalReached())
			{
				// 目標地点に到達した場合
				ROS_INFO("reach");
				_twist_pub.publish(cmd_vel);
				_nav_state = NavState::STANDBY;
				return;
			}
			_local_planner.computeVelocityCommands(cmd_vel); // ロボットの現在位置から次の位置までの移動に必要な速度を計算
			_twist_pub.publish(cmd_vel);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_navigation");
	MoveNavigation move_navigation;
	ros::spin();
	return 0;
}

// Arduinoからタイヤの角速度を受け取り、オドメトリを計算するプログラム
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <custom_msgs/TwoWDAngVel.h>

class Odometry
{
private:
	ros::NodeHandle _nh;
	ros::NodeHandle _pnh;
	ros::Subscriber _wheel_sub;
	ros::Publisher _odom_pub;
	ros::Publisher _vel_pub;
	tf2_ros::TransformBroadcaster _odom_broadcaster;
	tf2_ros::StaticTransformBroadcaster _laser_broadcaster;
	geometry_msgs::TransformStamped _tf_base;
	geometry_msgs::TransformStamped _tf_laser;
	nav_msgs::Odometry _odom;
	// geometry_msgs::Twist _vel;

	// オドメトリの計算に必要な変数
	double _radius = 0.1;	 // 車輪の半径
	double _tread = 0.3;	 // 車輪間の距離
	int _enc_cpr = 2048 * 4; // エンコーダのCPR値
	double _vx = 0.0;		 // 並進速度
	double _vth = 0.0;		 // 回転速度
	ros::Time last_time = ros::Time::now();
	double x = 0.0, y = 0.0, th = 0.0;

public:
	Odometry() : _nh(), _pnh("~")
	{
		_odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 50);
		_wheel_sub = _nh.subscribe("twoWD/wheelVel", 1, &Odometry::wheelCb, this);
		broadcastLaserTf();
		_pnh.param("/twoWD/radius", _radius);
		_pnh.param("/twoWD/tread", _tread);
		_pnh.param("/twoWD/enc_cpr", _enc_cpr);
	}
	// コールバック関数
	void wheelCb(const custom_msgs::TwoWDAngVel &wheelVel)
	{
		// タイヤの角速度から運動学計算により差動二輪ロボットの速度を計算
		_vx = _radius * (wheelVel.L + wheelVel.R) / 2;
		_vth = _radius * (wheelVel.R - wheelVel.L) / _tread;
	}

	// ロボットの姿勢をセット
	geometry_msgs::Pose setPose(double x, double y, double th)
	{
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = 0.0;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = sin(th / 2);
		pose.orientation.w = cos(th / 2);
		return pose;
	}

	// オドメトリの計算
	void getOdom()
	{
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		x += _vx * cos(th) * dt;
		y += _vx * sin(th) * dt;
		th += _vth * dt;

		// tf message
		_tf_base.header.stamp = current_time;
		_tf_base.header.frame_id = "odom";
		_tf_base.child_frame_id = "base_link";
		_tf_base.transform.translation.x = x;
		_tf_base.transform.translation.y = y;
		_tf_base.transform.translation.z = 0.0;
		tf2::Quaternion q_base;
		q_base.setRPY(0.0, 0.0, th); // RollPitchYawをクォータニオンに変換
		_tf_base.transform.rotation.x = q_base.x();
		_tf_base.transform.rotation.y = q_base.y();
		_tf_base.transform.rotation.z = q_base.z();
		_tf_base.transform.rotation.w = q_base.w();
		// broadcast tf
		_odom_broadcaster.sendTransform(_tf_base);

		// odom message
		_odom.header.stamp = current_time;
		_odom.header.frame_id = "odom";
		_odom.child_frame_id = "base_link";
		_odom.pose.pose = setPose(x, y, th);
		_odom.twist.twist.linear.x = _vx;
		_odom.twist.twist.linear.y = 0.0;
		_odom.twist.twist.angular.z = _vth;
		// publish odom
		_odom_pub.publish(_odom);

		last_time = current_time;
	}

	// laserのtfをセット(laserはbase_linkと反対向きに設置)
	void broadcastLaserTf()
	{
		_tf_laser.header.stamp = ros::Time::now();
		_tf_laser.header.frame_id = "base_link";
		_tf_laser.child_frame_id = "base_laser";
		_tf_laser.transform.translation.x = -0.2;
		_tf_laser.transform.translation.y = 0.0;
		_tf_laser.transform.translation.z = 0.0;
		tf2::Quaternion q_base;
		q_base.setRPY(0.0, 0.0, M_PI); // クォータニオンに変換
		_tf_laser.transform.rotation.x = q_base.x();
		_tf_laser.transform.rotation.y = q_base.y();
		_tf_laser.transform.rotation.z = q_base.z();
		_tf_laser.transform.rotation.w = q_base.w();
		// broadcast tf
		_laser_broadcaster.sendTransform(_tf_laser);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry");
	Odometry odometry;
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		odometry.getOdom();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TwistPublisher
{
public:
  TwistPublisher() : nh_(), pnh_("~")
  {
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy &joy_msg)
  {
    last_joy_ = joy_msg;
  }

  void timerCallback(const ros::TimerEvent &e)
  {
    // ジョイコンの割り当て(launchファイルから設定)
    int assign_x = 1;
    int assign_y = 0;
    int assign_z = 2;
    pnh_.getParam("/joy/assign_x", assign_x);
    pnh_.getParam("/joy/assign_y", assign_y);
    pnh_.getParam("/joy/assign_z", assign_z);

    int assign_x_straight = 7;
    int assign_y_straight = 6;
    pnh_.getParam("/joy/assign_x_straight", assign_x_straight);
    pnh_.getParam("/joy/assign_y_straight", assign_y_straight);

	// 最大速度の設定(launchファイルから設定)
    float max_x = 1.0;
    float max_y = 1.0;
    float max_z = 1.0;
    pnh_.getParam("/joy/max_x", max_x);
    pnh_.getParam("/joy/max_y", max_y);
    pnh_.getParam("/joy/max_z", max_z);

    geometry_msgs::Twist cmd_vel;
    if(0 <= assign_x_straight && assign_x_straight < last_joy_.axes.size()){
      cmd_vel.linear.x = max_x * last_joy_.axes[assign_x_straight];
    }
    if(0 <= assign_y_straight && assign_y_straight < last_joy_.axes.size()){
      cmd_vel.linear.y = max_y * last_joy_.axes[assign_y_straight];
    }
    if(0 <= assign_x && assign_x < last_joy_.axes.size()){
      if(cmd_vel.linear.x == 0)
        cmd_vel.linear.x = max_x * last_joy_.axes[assign_x];
    }
    if(0 <= assign_y && assign_y < last_joy_.axes.size()){
      if(cmd_vel.linear.y == 0)
        cmd_vel.linear.y = max_y * last_joy_.axes[assign_y];
    }
    if (0 <= assign_z && assign_z < last_joy_.axes.size())
    {
      cmd_vel.angular.z = max_z * last_joy_.axes[assign_z];
    }
    cmd_pub_.publish(cmd_vel);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  sensor_msgs::Joy last_joy_;
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

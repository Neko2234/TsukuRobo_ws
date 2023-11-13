#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <custom_msgs/ArmVel.h>

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
    int assign_arm = 4;
    pnh_.getParam("/joy/assign_x", assign_x);
    pnh_.getParam("/joy/assign_y", assign_y);
    pnh_.getParam("/joy/assign_z", assign_z);
    pnh_.getParam("/joy/assign_arm", assign_arm);

    // 最大速度の設定(launchファイルから設定)
    float max_x = 1.0;
    float max_y = 1.0;
    float max_z = 1.0;
    float max_arm = 1.0;
    pnh_.getParam("/joy/max_x", max_x);
    pnh_.getParam("/joy/max_y", max_y);
    pnh_.getParam("/joy/max_z", max_z);
    pnh_.getParam("/joy/max_arm", max_arm);

    geometry_msgs::Twist cmd_vel;
    if (0 <= assign_x && assign_x < last_joy_.axes.size())
    {
      cmd_vel.linear.x = max_x * last_joy_.axes[assign_x];
    }
    if (0 <= assign_y && assign_y < last_joy_.axes.size())
    {
      cmd_vel.linear.y = max_y * last_joy_.axes[assign_y];
    }
    if (0 <= assign_z && assign_z < last_joy_.axes.size())
    {
      cmd_vel.angular.z = max_z * last_joy_.axes[assign_z];
    }
    custom_msgs::ArmVel cmd_armvel;
    if(0 <= assign_arm && assign_arm < last_joy_.axes.size()){
      cmd_armvel.vel = max_arm * last_joy_.axes[assign_arm];
    }
    cmd_pub_.publish(cmd_vel);
    cmd_pub_.publish(cmd_armvel);
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

/**
 * @brief PowerBot teleoperation node based on joy tutorial at ros.org
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class PowerBotTeleop
{
 public:
  PowerBotTeleop();
  ~PowerBotTeleop();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  int linear_;
  int angular_;
  double linear_scale_;
  double angular_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

PowerBotTeleop::PowerBotTeleop() : linear_(1), angular_(0), linear_scale_(1.0), angular_scale_(0.5)
{
  /* Assign values from parameter server or use defaults */
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", angular_scale_, angular_scale_);
  nh_.param("scale_linear", linear_scale_, linear_scale_);
  /* Publish velocities to topic /RosAria/cmd_vel */
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
  /* Subscribe to joystick inputs */
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PowerBotTeleop::joyCallback, this);
}

PowerBotTeleop::~PowerBotTeleop()
{
}

void PowerBotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear_scale_ * joy->axes[linear_];
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = angular_scale_ * joy->axes[angular_];
  vel_pub_.publish(cmd_vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "csir_powerbot_teleop");
  PowerBotTeleop powerbot_teleop;
  ros::spin();
}

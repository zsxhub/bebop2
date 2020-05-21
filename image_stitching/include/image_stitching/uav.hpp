#ifndef UAV_HPP
#define UAV_HPP
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
#endif

#include <QImage>
#include <iostream>
class uav
{
public:
  uav();

  // 发布
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher cmd_pub;
  // 订阅
  ros::Subscriber state_sub;
  image_transport::Subscriber receiveImage_sub;
  // 变量
  cv::Mat receiveImage;//保存接受到的图像
  QImage ImageToQImage;//转换为QImage
  bool receiveImageFlag;//接受到图像为true 处理完设置为false

  //控制无人机的移动方向
  bool forward;
  bool backward;
  bool flayLeft;
  bool flayRight;
  bool flayUp;
  bool flayDown;
  bool turnLeft;
  bool turnRight;

  geometry_msgs::Twist cmd_vel;

//  ros::Rate *rate;

  void takeoff();
  void land();
  void cmd(float x,float y,float z);
  void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif // UAV_HPP

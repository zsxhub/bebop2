#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/image_stitching/uav3_node.hpp"
#include <QMatrix>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_stitching {

/*****************************************************************************
** Implementation
*****************************************************************************/

uav3::uav3(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {}

uav3::~uav3() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
}

bool uav3::init(std::string uavname)
{
  ros::init(init_argc,init_argv,"uav3_node");
  if ( ! ros::master::check() )
  {
    return false;
  }

//  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;// 第一次创建节点时会自动调用start()
  image_transport::ImageTransport it(nh);

  takeoff_pub= nh.advertise<std_msgs::Empty>(uavname + "/takeoff", 1);         // 发布 起飞命令
  land_pub   = nh.advertise<std_msgs::Empty>(uavname + "/land", 1);            // 发布 降落命令
  cmd_pub    = nh.advertise<geometry_msgs::Twist>(uavname + "/cmd_vel", 1);    // 发布 移动命令
  receiveImage_sub = it.subscribe(uavname + "/image_raw",5,&uav3::receiveImage_cb,this);// 订阅 图像信息

  //用于在gazebo仿真中测试
//  receiveImage_sub = it.subscribe("iris_2/camera_Monocular/image_raw",5,&uav3::receiveImage_cb,this);

  start();//开启线程 自动调用run()函数

  return true;
}

void uav3::run()
{
  ros::Rate loop_rate(20);
  int count = 0;

  while ( ros::ok() )
  {
//    if (receiveImageFlag == true)
//    {
//        receiveImageFlag = false;
//        std::cout << "emit signals!!" << std::endl;
//    }
    if(forward == true)   //前进
      cmd(0.2,0,0);
    if(backward == true)  //后退
      cmd(-0.2,0,0);
    if(flayLeft == true)  //左飞
      cmd(0,0.2,0);
    if(flayRight == true) //右飞
      cmd(0,-0.2,0);
    if(flayUp == true)    //上升
      cmd(0,0,0.1);
    if(flayDown == true)  //下降
      cmd(0,0,-0.1);

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


//接受uav1的图像
void uav3::receiveImage_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        receiveImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8)->image;
        Q_EMIT uav3RgbimageSignal(receiveImage);
        ImageToQImage = QImage(receiveImage.data,receiveImage.cols,receiveImage.rows,receiveImage.step[0],QImage::Format_RGB888);
        Q_EMIT showUav3ImageSignal(ImageToQImage);
        receiveImageFlag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "sub1Image_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << std::endl;
    }
}

}  // namespace image_stitching

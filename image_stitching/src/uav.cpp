#include "../include/image_stitching/uav.hpp"

using namespace std;
uav::uav()
{
  receiveImageFlag = false;
  forward = false;
  backward = false;
  flayLeft = false;
  flayRight = false;
  flayUp = false;
  flayDown = false;
  turnLeft = false;
  turnRight = false;
}

void uav::takeoff()
{
    takeoff_pub.publish(std_msgs::Empty());
    cout << "bebop  takeoff! " << endl;
//    ROS_INFO("bebop  takeoff! ");
}
void uav::land()
{
    land_pub.publish(std_msgs::Empty());
    cout << "bebop  land! " << endl;
//    ROS_INFO("bebop  land! ");
}
void uav::cmd(float x,float y,float z)
{
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.linear.z = z;
    cmd_pub.publish(cmd_vel);
    cout << "bebop  send cmd! " << endl;
//    ROS_INFO("bebop  send cmd! ");
}


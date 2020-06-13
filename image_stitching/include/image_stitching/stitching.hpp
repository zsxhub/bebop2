#ifndef STITCHING_HPP
#define STITCHING_HPP

#ifndef Q_MOC_RUN
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//#include <ros/ros.h>

#endif

#include <iostream>
#include <QThread>
#include <QImage>
#include <QMutex>
#include <QWaitCondition>
#include <QTime>

#include "myORBextractor.hpp"

namespace image_stitching{

class stitching : public QThread
{
  Q_OBJECT
public:
  stitching();


  /* 用于控制拼接进程 */
  mutable QMutex stitching_mutex_;
  QWaitCondition imageRecOK;

  /* true --> 执行拼接程序 */
  bool isStitching;
  bool stitchingThreadStatue;
  typedef struct
  {
      cv::Point2f left_top;
      cv::Point2f left_bottom;
      cv::Point2f right_top;
      cv::Point2f right_bottom;
  }four_corners_t;
  four_corners_t corners;

  cv::Mat leftImage,rightImage,stitchingImage;
  bool leftImageRec_flag,rightImageRec_flag;

  void run();

      //优化两图的连接处，使得拼接自然
      void OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst);

      void CalcCorners( cv::Mat H,  cv::Mat src);
      cv::Mat imageProcess(cv::Mat img1, cv::Mat img2);
      void drawText(cv::Mat & image);

public Q_SLOTS:
      void deal_uav1RgbimageSignal(cv::Mat image);
      void deal_uav2RgbimageSignal(cv::Mat image);

Q_SIGNALS://Qt信号
      void showStitchingImageSignal(QImage);

};

}
#endif // STITCHING_HPP


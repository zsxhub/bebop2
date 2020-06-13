#include "../include/image_stitching/stitching.hpp"

#define nFeatures       500    //图像金字塔上特征点的数量
#define nLevels         8       //图像金字塔层数
#define myfScaleFactor    1.2     //金字塔比例因子
#define myfIniThFAST      20      //检测fast角点阈值
#define myfMinThFAST      8       //最低阈值

namespace image_stitching{

using namespace cv;
using namespace std;
stitching::stitching()
{
  leftImageRec_flag = false;
  rightImageRec_flag = false;
  isStitching = false;
  stitchingThreadStatue = true;
//  start();
}


void stitching::run()
{
  while (stitchingThreadStatue) {

    if(isStitching == true)
    {
      stitching_mutex_.lock();

      if(leftImageRec_flag == false)
        imageRecOK.wait(&stitching_mutex_);
      if(rightImageRec_flag == false)
        imageRecOK.wait(&stitching_mutex_);

      leftImageRec_flag = false;
      rightImageRec_flag = false;
      stitchingImage = imageProcess(leftImage,rightImage);
      std::cout << "receive image ok " << std::endl;
      try
      {
          cvtColor(stitchingImage, stitchingImage, CV_BGR2RGB);
          QImage ImageToQImage = QImage(stitchingImage.data,stitchingImage.cols,stitchingImage.rows,stitchingImage.step[0],QImage::Format_RGB888);
          Q_EMIT showStitchingImageSignal(ImageToQImage);
      }
      catch (...)
      {
        std::cout << " stitchingImage could not convert " << std::endl;
      }
      stitching_mutex_.unlock();
    }
    else
    {
      std::cout << "thread is running, isStitching = false " << std::endl;
      msleep(500);
    }

  }
}

void stitching::deal_uav1RgbimageSignal(cv::Mat image)
{

  try
  {
    cvtColor(image, leftImage, CV_RGB2BGR);
    leftImageRec_flag = true;
    //唤醒拼接线程
    imageRecOK.wakeAll();
//    std::cout << "cvtColor ok " << std::endl;
  }
  catch(...)
  {
    std::cout << " deal_uav1RgbimageSignal could not convert " << std::endl;
  }
}

void stitching::deal_uav2RgbimageSignal(cv::Mat image)
{
  try
  {
    cvtColor(image, rightImage, CV_RGB2BGR);
    rightImageRec_flag = true;
    imageRecOK.wakeAll();
  }
  catch(...)
  {
    std::cout << " deal_uav2RgbimageSignal could not convert " << std::endl;
  }
}
void stitching::CalcCorners( cv::Mat H,  cv::Mat src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    // cout << "V2: " << V2 << endl;
    // cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}

cv::Mat stitching::imageProcess(cv::Mat img1, cv::Mat img2)
{
    //灰度图转换
    Mat grayImage1, grayImage2;
    cvtColor(img1, grayImage1, CV_RGB2GRAY);
    cvtColor(img2, grayImage2, CV_RGB2GRAY);


    //提取特征点
    Mat imageDesc1, imageDesc2;
    vector<KeyPoint> keyPoint1, keyPoint2;

    image_stitching::ORBextractor ORBextractor =  image_stitching::ORBextractor(nFeatures,myfScaleFactor,nLevels,myfIniThFAST,myfMinThFAST);
    ORBextractor(grayImage1, cv::Mat(), keyPoint1, imageDesc1);
    ORBextractor(grayImage2, cv::Mat(), keyPoint2, imageDesc2);

    // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create();
    BFMatcher matcher(NORM_HAMMING);
    // FlannBasedMatcher matcher;
    vector< vector<DMatch> > matches;
    vector<DMatch> good_matches;

    // 会为每个目标特征点返回两个最匹配的两个特征点
    matcher.knnMatch(imageDesc1,imageDesc2,matches,2);
    // 筛选

    for (int i = 0; i < matches.size(); i++)
    {
        // 比较欧氏距离差距
        if (matches[i][0].distance < 0.6 * matches[i][1].distance)
        {
            good_matches.push_back(matches[i][0]);
        }
    }
    cout << "matches:" << matches.size() << endl;
    cout << "good_matches:" << good_matches.size() << endl;

    Mat img_goodmatch;
//////    try{
    if(good_matches.size()>=1)
        drawMatches(img1, keyPoint1, img2, keyPoint2, good_matches, img_goodmatch);
    else
        img_goodmatch = img1;
//    cv::imshow("Matches",img_goodmatch);

    Mat homo;
    Mat dst(img1.rows, img1.cols+img2.cols, CV_8UC3);
    dst.setTo(0);

    if(good_matches.size()>=8)
    {
        vector<Point2f> imagePoints1, imagePoints2;
        for (int i = 0; i<good_matches.size(); i++)
        {
            imagePoints2.push_back(keyPoint2[good_matches[i].trainIdx].pt);
            imagePoints1.push_back(keyPoint1[good_matches[i].queryIdx].pt);
        }

        //获取图像1到图像2的投影映射矩阵 尺寸为3*3
        homo = findHomography(imagePoints2, imagePoints1, CV_RANSAC);


    //计算配准图的四个顶点坐标
        CalcCorners(homo, img2);
        //图像配准
        Mat imageTransform2;
    /*
        https://blog.csdn.net/baidu_38172402/article/details/82783663
        void cv::warpPerspective(
            cv::InputArray src,                             // 输入图像
            cv::OutputArray dst,                            // 输出图像
            cv::InputArray M,                               // 3x3 变换矩阵
            cv::Size dsize,                                 // 目标图像大小
            int flags = cv::INTER_LINEAR,                   // 插值方法
            int borderMode = cv::BORDER_CONSTANT,           // 外推方法
            const cv::Scalar& borderValue = cv::Scalar()    //常量边界时使用
        );
    */
        warpPerspective(img2, imageTransform2, homo, Size(img1.cols+img2.cols, img1.rows));

        imageTransform2.copyTo(dst(Rect(0, 0, imageTransform2.cols, imageTransform2.rows)));

        img1.copyTo(dst(Rect(0, 0, img1.cols, img1.rows)));

        OptimizeSeam(img1, imageTransform2, dst);

    }
    else//匹配点不足８个
    {
        img1.copyTo(dst(Rect(0, 0, img1.cols, img1.rows)));
        drawText(dst);
        cout << " Insufficient feature points, unable to realize image stitching " << endl;
    }
    return dst;

//    }
//    catch(...){
//        cout << "error!!!!" << endl;
//        return img1;
//    }

}


//优化两图的连接处，使得拼接自然
void stitching::OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst)
{
    int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界
    if(start < 0)
        start = 0;
    double processWidth = img1.cols - start;//重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; //注意，是列数*通道数
    double alpha = 1;//img1中像素的权重
    cout << "processWidth: " << processWidth << endl;
    cout << "start: " << start << endl;

    for (int i = 0; i < rows; i++)
    {
        uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

        }
    }

}

void stitching::drawText(cv::Mat & image)
{
    putText(image, "Insufficient feature points, unable to realize image stitching",
            Point(0, image.rows/2),
            FONT_HERSHEY_COMPLEX, 1, // font face and scale
            Scalar(0, 0, 255), // red (bgr)
            1, LINE_AA); // line thickness and type
}

}

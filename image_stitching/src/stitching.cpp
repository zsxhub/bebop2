#include "../include/image_stitching/stitching.hpp"

#define nFeatures       2000    //图像金字塔上特征点的数量
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
  middleImageRec_flag = false;
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
      if(middleImageRec_flag == false)
        imageRecOK.wait(&stitching_mutex_);
      if(rightImageRec_flag == false)
        imageRecOK.wait(&stitching_mutex_);

      leftImageRec_flag = false;
      middleImageRec_flag = false;
      rightImageRec_flag = false;

      stitchingImage = stitchingThreeImage(leftImage, middleImage,rightImage);
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
    cvtColor(image, middleImage, CV_RGB2BGR);
    rightImageRec_flag = true;
    imageRecOK.wakeAll();
  }
  catch(...)
  {
    std::cout << " deal_uav2RgbimageSignal could not convert " << std::endl;
  }
}
void stitching::deal_uav3RgbimageSignal(cv::Mat image)
{
  try
  {
    cvtColor(image, rightImage, CV_RGB2BGR);
    rightImageRec_flag = true;
    imageRecOK.wakeAll();
  }
  catch(...)
  {
    std::cout << " deal_uav3RgbimageSignal could not convert " << std::endl;
  }
}
void stitching::CalcCorners( cv::Mat H,  cv::Mat src, four_corners_t &corners)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;

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

cv::Mat stitching::stitchingThreeImage(cv::Mat img1, cv::Mat img2, cv::Mat img3)
{

    Mat dst(img2.rows*2,img2.cols*3, CV_8UC3);
    dst.setTo(0);
    Point2f offfset;
    offfset.y = img2.rows/2;
    offfset.x = img2.cols;
    int cols,rows;
    //灰度图转换
    Mat grayImage1, grayImage2, grayImage3;
    cvtColor(img1, grayImage1, CV_RGB2GRAY);
    cvtColor(img2, grayImage2, CV_RGB2GRAY);
    cvtColor(img3, grayImage3, CV_RGB2GRAY);

    Mat imageDesc1, imageDesc2,imageDesc3;
    vector<KeyPoint> keyPoint1, keyPoint2,keyPoint3;


    image_stitching::ORBextractor ORBextractor =  image_stitching::ORBextractor(nFeatures,myfScaleFactor,nLevels,myfIniThFAST,myfMinThFAST);
    ORBextractor(grayImage1, cv::Mat(), keyPoint1, imageDesc1);
    ORBextractor(grayImage2, cv::Mat(), keyPoint2, imageDesc2);
    ORBextractor(grayImage3, cv::Mat(), keyPoint3, imageDesc3);

    BFMatcher matcher(NORM_HAMMING);
    // matches_LM --> 左图和中图   matches_MR --> 中图和右图
    vector< vector<DMatch> > matches_LM,matches_MR;
    vector<DMatch> good_matches_LM,good_matches_MR;

    // 会为每个目标特征点返回两个最匹配的两个特征点
    matcher.knnMatch(imageDesc1,imageDesc2,matches_LM,2);
    matcher.knnMatch(imageDesc2,imageDesc3,matches_MR,2);
    // 筛选
    for (int i = 0; i < matches_LM.size(); i++)
    {
        // 比较欧氏距离差距
        if(matches_LM[i][0].distance < 0.6 * matches_LM[i][1].distance)
            good_matches_LM.push_back(matches_LM[i][0]);
    }
    for (int i = 0; i < matches_MR.size(); i++)
    {
        // 比较欧氏距离差距
        if(matches_MR[i][0].distance < 0.5 * matches_MR[i][1].distance)
            good_matches_MR.push_back(matches_MR[i][0]);
    }
    try
    {
        cv::Mat homo_LM,homo_MR;
        cv::Mat rightImageTransform;
        cv::Mat leftImageTransform;
        cv::Mat middleImageTransform(img2.rows*2,img2.cols*3, CV_8UC3);
        middleImageTransform.setTo(0);
        bool leftTransform_ok = false,rightTransform_ok = false;

        img2.copyTo(dst(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));
        img2.copyTo(middleImageTransform(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));

        if(good_matches_MR.size()>=8)
        {
            vector<Point2f> imagePoints1, imagePoints2;
            for (int i = 0; i<good_matches_MR.size(); i++)
            {
                imagePoints1.push_back(keyPoint2[good_matches_MR[i].queryIdx].pt + offfset);
                imagePoints2.push_back(keyPoint3[good_matches_MR[i].trainIdx].pt);
            }

            //获取图像2到图像1的投影映射矩阵 尺寸为3*3
            homo_MR = findHomography(imagePoints2, imagePoints1, CV_RANSAC);
            //计算配准图的四个顶点坐标
            CalcCorners(homo_MR, img3, rightImg_corners);
            //图像配准
            warpPerspective(img3, rightImageTransform, homo_MR, Size(MAX(rightImg_corners.right_top.x,rightImg_corners.right_bottom.x),
                                                                     MAX(rightImg_corners.left_bottom.y,rightImg_corners.right_bottom.y)));
            if(rightImageTransform.cols>=dst.cols) cols = dst.cols;
            else cols = rightImageTransform.cols;
            if(rightImageTransform.rows>=dst.rows) rows = dst.rows;
            else rows = rightImageTransform.rows;
            Point2i start;
            start.x = MAX(0, MIN(rightImg_corners.left_top.x, rightImg_corners.left_bottom.x));
            start.y = MAX(0, MIN(rightImg_corners.left_top.y, rightImg_corners.right_top.y));
            cv::Rect rect(start.x, start.y, cols-start.x, rows-start.y);
            cv::Mat rect_rightImageTransform = rightImageTransform(rect);
            rect_rightImageTransform.copyTo(dst(rect));
            rightTransform_ok = true;

            middleImg_corners.left_top.x = offfset.x;
            middleImg_corners.left_top.y = offfset.y;
            middleImg_corners.right_bottom.x = offfset.x + img2.cols;
            middleImg_corners.right_bottom.y = offfset.y + img2.rows;
            OptimizeSeam(middleImageTransform, rightImageTransform, dst, middleImg_corners, rightImg_corners);

            overlap_rate_right =  (float)(middleImg_corners.right_bottom.x - (rightImg_corners.left_top.x+rightImg_corners.left_bottom.x)/2.0) * 100 / (float)img2.cols;
            middleImg_corners.left_top.x = MIN(middleImg_corners.left_top.x, rightImg_corners.left_top.x);
            middleImg_corners.left_top.y = MIN(middleImg_corners.left_top.y, rightImg_corners.left_top.y);
            middleImg_corners.right_bottom.x = MAX(middleImg_corners.right_bottom.x, rightImg_corners.right_bottom.x);
            middleImg_corners.right_bottom.y = MAX(middleImg_corners.right_bottom.y, rightImg_corners.right_bottom.y);
            dst.copyTo(middleImageTransform);
        }
        else//匹配点不足８个
        {
            cout << " Insufficient feature points, unable to realize right image stitching " << endl;
        }

        if(good_matches_LM.size()>=8)
        {
            vector<Point2f> imagePoints1, imagePoints2;
            for (int i = 0; i<good_matches_LM.size(); i++)
            {
                imagePoints1.push_back(keyPoint1[good_matches_LM[i].queryIdx].pt);
                imagePoints2.push_back(keyPoint2[good_matches_LM[i].trainIdx].pt + offfset);
            }

            //获取图像2到图像1的投影映射矩阵 尺寸为3*3
            homo_LM = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
            //计算配准图的四个顶点坐标
            CalcCorners(homo_LM, img1, leftImg_corners);
            //图像配准

            warpPerspective(img1, leftImageTransform, homo_LM,Size(MAX(leftImg_corners.right_top.x,leftImg_corners.right_bottom.x),
                                                                   MAX(leftImg_corners.left_bottom.y,leftImg_corners.right_bottom.y)));
            if(leftImageTransform.cols>dst.cols) cols = dst.cols-1;
            else cols = leftImageTransform.cols;
            if(leftImageTransform.rows>dst.rows) rows = dst.rows-1;
            else rows = leftImageTransform.rows;
            Point2i start;
            start.x = MAX(0, MIN(leftImg_corners.left_top.x, leftImg_corners.left_bottom.x));
            start.y = MAX(0, MIN(leftImg_corners.left_top.y, leftImg_corners.right_top.y));
            cv::Rect rect(start.x, start.y, cols-start.x, rows-start.y);
            cv::Mat rect_leftImageTransform = leftImageTransform(rect);
            rect_leftImageTransform.copyTo(dst(rect));
            leftTransform_ok = true;

            OptimizeSeam(middleImageTransform, leftImageTransform, dst, middleImg_corners, leftImg_corners);
            overlap_rate_left =  (float)((leftImg_corners.right_top.x+leftImg_corners.right_bottom.x)/2.0 - offfset.x) * 100 / (float)img2.cols;
        }
        else//匹配点不足８个
        {
            cout << " Insufficient feature points, unable to realize left image stitching " << endl;
        }
        drawText(dst, overlap_rate_left, Point(offfset.x, offfset.y));
        drawText(dst, overlap_rate_right, Point(offfset.x + img2.cols, offfset.y));
        return dst;
    }
    catch(...)
    {
        cout << "error!!!!" << endl;
        img2.copyTo(dst(Rect(offfset.x, offfset.y, img2.cols, img2.rows)));
        return dst;
    }

}


//优化两图的连接处，使得拼接自然
void stitching::OptimizeSeam(cv::Mat& sourceImage, cv::Mat& transImage, cv::Mat& dst,four_corners_t source_corners,four_corners_t trans_corners)
{

    Point2i source, trans;  // 临时保存顶点坐标值

    //计算重叠区域左上角坐标
    source.x = MIN(source_corners.left_top.x, source_corners.left_bottom.x);
    source.y = MIN(source_corners.left_top.y, source_corners.right_top.y);
    trans.x = MIN(trans_corners.left_top.x, trans_corners.left_bottom.x);
    trans.y = MIN(trans_corners.left_top.y, trans_corners.right_top.y);
    overlap.left_top.x = MAX(MAX(source.x, trans.x), 0); // 限制必须大于0
    overlap.left_top.y = MAX(MAX(source.y, trans.y), 0); // 限制必须大于0
    //计算重叠区域右下角坐标
    source.x = MAX(source_corners.right_top.x, source_corners.right_bottom.x);
    source.y = MAX(source_corners.left_bottom.y, source_corners.right_bottom.y);
    trans.x = MAX(trans_corners.right_top.x, trans_corners.right_bottom.x);
    trans.y = MAX(trans_corners.left_bottom.y, trans_corners.right_bottom.y);
    overlap.right_bottom.x = MIN(MIN(source.x, trans.x), dst.cols);  // 限制必须小于 dst.cols
    overlap.right_bottom.y = MIN(MIN(source.y, trans.y), dst.rows);  // 限制必须小于 dst.rows

    double overlapWidth =overlap.right_bottom.x - overlap.left_top.x ;//重叠区域的宽度
    cout << "overlapWidth " << overlapWidth << endl;
    double alpha = 1;//img1中像素的权重
    if(overlapWidth > 0)
    {
        for (int i = overlap.left_top.y; i < overlap.right_bottom.y; i++)
        {
            uchar* p = sourceImage.ptr<uchar>(i);  //获取第i行的首地址
            uchar* t = transImage.ptr<uchar>(i);
            uchar* d = dst.ptr<uchar>(i);
            for (int j = overlap.left_top.x; j < overlap.right_bottom.x; j++)
            {
                //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
                if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
                {
                    alpha = 1;
                }
                else if(p[j * 3] == 0 && p[j * 3 + 1] == 0 && p[j * 3 + 2] == 0)
                {
                    alpha = 0;
                }
                else
                {
                    //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                    alpha = (overlapWidth - (j - overlap.left_top.x)) / overlapWidth;
                }

                d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
                d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
                d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

            }
        }
    }
}

void stitching::drawText(cv::Mat & image, float rate, cv::Point centerpoint)
{
    string text = to_string(rate).substr(0,5);
    text = text + "%";
    putText(image, text,
            Point(centerpoint.x, centerpoint.y),
            FONT_HERSHEY_COMPLEX, 3,    // font face and scale
            Scalar(0, 0, 255),          // red (bgr)
            2, LINE_AA);                // line thickness and type
}

}

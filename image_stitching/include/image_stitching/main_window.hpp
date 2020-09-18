/**
 * @file /include/image_stitching/main_window.hpp
 *
 * @brief Qt based gui for image_stitching.
 *
 * @date November 2010
 **/
#ifndef image_stitching_MAIN_WINDOW_H
#define image_stitching_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include <QMutex>
#include <QWaitCondition>

#include "uav1_node.hpp"
#include "uav2_node.hpp"
#include "uav3_node.hpp"
#include "moveuav_window.hpp"
#include "stitching.hpp"

namespace image_stitching {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();



  uav1 uav1Node;
  uav2 uav2Node;
  uav3 uav3Node;

  moveUav *moveUav1;
  moveUav *moveUav2;
  moveUav *moveUav3;

  stitching *imageStitching;

  QImage uav1Image;
  QImage uav2Image;
  QImage uav3Image;

  QImage stitchingImage;
  std::string uav1Name;
  std::string uav2Name;
  std::string uav3Name;

  /*用来保护一个对象、数据结构、代码段、使得它们在同一一时刻，只有一个线程有访问权限*/
  mutable QMutex uav1Image_mutex_;
  mutable QMutex uav2Image_mutex_;
  mutable QMutex uav3Image_mutex_;
  mutable QMutex stitchingImage_mutex_;



	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

  void displayUav1Image(const QImage image);
  void displayUav2Image(const QImage image);
  void displayUav3Image(const QImage image);
  void displayStitchingImage(const QImage image);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

  void deal_showUav1ImageSignal(QImage image);
  void deal_showUav2ImageSignal(QImage image);
  void deal_showUav3ImageSignal(QImage image);
  void deal_showStitchingImageSignal(QImage image);

  void on_uav1Takeoff_pBtn_clicked();
  void on_uav1Land_pBtn_clicked();
  void on_uav1Connect_pBtn_clicked();
  void on_uav1Move_pBtn_clicked();
  void on_uav1ShowImage_pBtn_clicked();

  void on_uav2Takeoff_pBtn_clicked();
  void on_uav2Land_pBtn_clicked();
  void on_uav2Connect_pBtn_clicked();
  void on_uav2Move_pBtn_clicked();
  void on_uav2ShowImage_pBtn_clicked();

  void on_uav3Takeoff_pBtn_clicked();
  void on_uav3Land_pBtn_clicked();
  void on_uav3Connect_pBtn_clicked();
  void on_uav3Move_pBtn_clicked();
  void on_uav3ShowImage_pBtn_clicked();

  void on_stitching_checkBox_stateChanged(int arg1);

  void deal_forwardSignal(int UAVx, bool state);
  void deal_backwardSignal(int UAVx, bool state);
  void deal_flayLeftSignal(int UAVx, bool state);
  void deal_flayRightSignal(int UAVx, bool state);
  void deal_flayUpSignal(int UAVx, bool state);
  void deal_flayDownSignal(int UAVx, bool state);
  void deal_turnLeftSignal(int UAVx, bool state);
  void deal_turnRightSignal(int UAVx, bool state);


private:
	Ui::MainWindowDesign ui;

protected:
//  void paintEvent(QPaintEvent *);



};

}  // namespace image_stitching

#endif // image_stitching_MAIN_WINDOW_H

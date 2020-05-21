/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/image_stitching/main_window.hpp"
#include <QDebug>
/*****************************************************************************
** Namespaces
*****************************************************************************/

/*
 * 应用于 on_uav1Connect_pBtn_clicked()和on_uav2Connect_pBtn_clicked()
 * 0 --> 不运行bash命令
 * 1 --> 运行bash命令
 */
#define runCommand 1

namespace image_stitching {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , uav1Node(argc,argv)
  , uav2Node(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  moveUav1 = new moveUav(1);
  moveUav2 = new moveUav(2);

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
  QObject::connect(&uav1Node, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&uav1Node,SIGNAL(showUav1ImageSignal(QImage)),this,SLOT(deal_showUav1ImageSignal(QImage)));
  QObject::connect(&uav2Node, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&uav2Node,SIGNAL(showUav2ImageSignal(QImage)),this,SLOT(deal_showUav2ImageSignal(QImage)));

  QObject::connect(moveUav1,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(moveUav1,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));

  QObject::connect(moveUav2,SIGNAL(forwardSignal(int,bool)),this,SLOT(deal_forwardSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(backwardSignal(int,bool)),this,SLOT(deal_backwardSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayLeftSignal(int,bool)),this,SLOT(deal_flayLeftSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayRightSignal(int,bool)),this,SLOT(deal_flayRightSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayUpSignal(int,bool)),this,SLOT(deal_flayUpSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(flayDownSignal(int,bool)),this,SLOT(deal_flayDownSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(turnLeftSignal(int,bool)),this,SLOT(deal_turnLeftSignal(int,bool)));
  QObject::connect(moveUav2,SIGNAL(turnRightSignal(int,bool)),this,SLOT(deal_turnRightSignal(int,bool)));

    uav1Name = "bebop4";//默认值
    ui.uav1Land_pBtn->setEnabled(false);
    ui.uav1Takeoff_pBtn->setEnabled(false);
    ui.uav1Move_pBtn->setEnabled(false);
    ui.uav1ShowImage_pBtn->setEnabled(false);

    uav2Name = "bebop1";//默认值
    ui.uav2Land_pBtn->setEnabled(false);
    ui.uav2Takeoff_pBtn->setEnabled(false);
    ui.uav2Move_pBtn->setEnabled(false);
    ui.uav2ShowImage_pBtn->setEnabled(false);
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
  moveUav2->close();
  moveUav1->close();
  QMainWindow::closeEvent(event);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


/*****************************  无人机1 *****************************/

void MainWindow::displayUav1Image(const QImage image)
{
  uav1Image_mutex_.lock();
  uav1Image = image.copy();
  ui.uav1Image_label->setPixmap(QPixmap::fromImage(uav1Image));
//  ui.uav1Image_label->resize(ui.uav1Image_label->pixmap()->size());
  uav1Image_mutex_.unlock();

}

void MainWindow::deal_showUav1ImageSignal(QImage image)
{
  static int i=0;
  qDebug() << "deal_showUav1ImageSignal"<<i;
  i++;
  displayUav1Image(image);
}

void MainWindow::on_uav1Takeoff_pBtn_clicked()
{
  uav1Node.takeoff();
}

void MainWindow::on_uav1Land_pBtn_clicked()
{
  uav1Node.land();
}

void MainWindow::on_uav1Connect_pBtn_clicked()
{
  uav1Name = ui.uav1Name_cBox->currentText().toStdString();

#if runCommand
  std::string str = "gnome-terminal -x bash -c 'roslaunch bebop_driver "
                        + uav1Name
                        +"_node.launch'&";
  const char *command = str.c_str();
  system(command);
#endif

  ui.uav1Takeoff_pBtn->setEnabled(true);
  ui.uav1Land_pBtn->setEnabled(true);
  ui.uav1Move_pBtn->setEnabled(true);
  ui.uav1ShowImage_pBtn->setEnabled(true);
}

void MainWindow::on_uav1Move_pBtn_clicked()
{
  moveUav1->show();
}

void MainWindow::on_uav1ShowImage_pBtn_clicked()
{
  if ( !uav1Node.init(uav1Name) )
  {
    showNoMasterMessage();
  }
  else
  {
    ui.uav1ShowImage_pBtn->setEnabled(false);
  }
}

/*****************************  无人机2 *****************************/

void MainWindow::deal_showUav2ImageSignal(QImage image)
{
//  static int i=0;
//  qDebug() << "deal_showUav1ImageSignal"<<i;
//  i++;
  displayUav2Image(image);
}
void MainWindow::displayUav2Image(const QImage image)
{
  uav2Image_mutex_.lock();
  uav2Image = image.copy();
  ui.uav2Image_label->setPixmap(QPixmap::fromImage(uav2Image));
//  ui.uav2Image_label->resize(ui.uav2Image_label->pixmap()->size());
  uav2Image_mutex_.unlock();
}
void MainWindow::on_uav2Takeoff_pBtn_clicked()
{
  uav2Node.takeoff();
}

void MainWindow::on_uav2Land_pBtn_clicked()
{
  uav2Node.land();
}

void MainWindow::on_uav2Connect_pBtn_clicked()
{
  uav2Name = ui.uav2Name_cBox->currentText().toStdString();

#if runCommand
  std::string str = "gnome-terminal -x bash -c 'roslaunch bebop_driver "
                        + uav2Name
                        +"_node.launch'&";
  const char *command = str.c_str();


  system(command);
#endif

  ui.uav2Takeoff_pBtn->setEnabled(true);
  ui.uav2Land_pBtn->setEnabled(true);
  ui.uav2Move_pBtn->setEnabled(true);
  ui.uav2ShowImage_pBtn->setEnabled(true);
}

void MainWindow::on_uav2Move_pBtn_clicked()
{
  moveUav2->show();
}

void MainWindow::on_uav2ShowImage_pBtn_clicked()
{
  if ( !uav2Node.init(uav2Name) )
  {
    showNoMasterMessage();
  }
  else
  {
    ui.uav2ShowImage_pBtn->setEnabled(false);
  }
}

/*****************************  控制无人机的移动 *****************************/
void MainWindow::deal_forwardSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.forward = state;
  else if(UAVx == 2)
    uav2Node.forward = state;
}
void MainWindow::deal_backwardSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.backward = state;
  else if(UAVx == 2)
    uav2Node.backward = state;

}
void MainWindow::deal_flayLeftSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayLeft = state;
  else if(UAVx == 2)
    uav2Node.flayLeft = state;
}
void MainWindow::deal_flayRightSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayRight = state;
  else if(UAVx == 2)
    uav2Node.flayRight = state;
}
void MainWindow::deal_flayUpSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayUp = state;
  else if(UAVx == 2)
    uav2Node.flayUp = state;
}
void MainWindow::deal_flayDownSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.flayDown = state;
  else if(UAVx == 2)
    uav2Node.flayDown = state;
}
void MainWindow::deal_turnLeftSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.turnLeft = state;
  else if(UAVx == 2)
    uav2Node.turnLeft = state;
}
void MainWindow::deal_turnRightSignal(int UAVx, bool state)
{
  if(UAVx == 1)
    uav1Node.turnRight = state;
  else if(UAVx == 2)
    uav2Node.turnRight = state;
}


}  // namespace image_stitching


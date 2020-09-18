#include "../include/image_stitching/moveuav_window.hpp"
#include "ui_moveuav.h"


moveUav::moveUav(int slelcet, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::moveUav)
{
  ui->setupUi(this);

  UAVxControl = slelcet;

  if(slelcet == 1)
    this->setWindowTitle("UAV1 control");
  if(slelcet == 2)
    this->setWindowTitle("UAV2 control");
  if(slelcet == 3)
    this->setWindowTitle("UAV3 control");
}

moveUav::~moveUav()
{
  delete ui;
}

void moveUav::on_forward_pBtn_pressed()
{
    Q_EMIT forwardSignal(UAVxControl,true);
}

void moveUav::on_forward_pBtn_released()
{
    Q_EMIT forwardSignal(UAVxControl,false);
}

void moveUav::on_backward_pBtn_pressed()
{
  Q_EMIT backwardSignal(UAVxControl,true);
}

void moveUav::on_backward_pBtn_released()
{
  Q_EMIT backwardSignal(UAVxControl,false);
}

void moveUav::on_flayLeft_pBtn_pressed()
{
  Q_EMIT flayLeftSignal(UAVxControl,true);
}

void moveUav::on_flayLeft_pBtn_released()
{
  Q_EMIT flayLeftSignal(UAVxControl,false);
}

void moveUav::on_flayRight_pBtn_pressed()
{
  Q_EMIT flayRightSignal(UAVxControl,true);
}

void moveUav::on_flayRight_pBtn_released()
{
  Q_EMIT flayRightSignal(UAVxControl,false);
}

void moveUav::on_up_pBtn_pressed()
{
  Q_EMIT flayUpSignal(UAVxControl,true);
}

void moveUav::on_up_pBtn_released()
{
  Q_EMIT flayUpSignal(UAVxControl,false);
}

void moveUav::on_down_pBtn_pressed()
{
  Q_EMIT flayDownSignal(UAVxControl,true);
}

void moveUav::on_down_pBtn_released()
{
  Q_EMIT flayDownSignal(UAVxControl,false);
}

void moveUav::on_turnLeft_pBtn_pressed()
{
  Q_EMIT turnLeftSignal(UAVxControl,true);
}

void moveUav::on_turnLeft_pBtn_released()
{
  Q_EMIT turnLeftSignal(UAVxControl,false);
}

void moveUav::on_turnRight_pBtn_pressed()
{
  Q_EMIT turnRightSignal(UAVxControl,true);
}

void moveUav::on_turnRight_pBtn_released()
{
  Q_EMIT turnRightSignal(UAVxControl,false);
}

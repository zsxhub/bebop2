#ifndef MOVEUAV_HPP
#define MOVEUAV_HPP

#include <QWidget>
#include "ui_moveuav.h"
#include <QThread>

namespace Ui {
class moveUav;
}
//namespace image_stitching {
class moveUav : public QWidget
{
  Q_OBJECT

public:
  explicit moveUav(int slelcet, QWidget *parent = 0);
  ~moveUav();

  int UAVxControl; // 1 --> UAV1  2 --> UAV2


Q_SIGNALS:
  void forwardSignal(int,bool);
  void backwardSignal(int,bool);
  void flayLeftSignal(int,bool);
  void flayRightSignal(int,bool);
  void flayUpSignal(int,bool);
  void flayDownSignal(int,bool);
  void turnLeftSignal(int,bool);
  void turnRightSignal(int,bool);

public Q_SLOTS:
  void on_forward_pBtn_pressed();

  void on_forward_pBtn_released();

  void on_backward_pBtn_pressed();

  void on_backward_pBtn_released();

  void on_flayLeft_pBtn_pressed();

  void on_flayLeft_pBtn_released();

  void on_flayRight_pBtn_pressed();

  void on_flayRight_pBtn_released();

  void on_up_pBtn_pressed();

  void on_up_pBtn_released();

  void on_down_pBtn_pressed();

  void on_down_pBtn_released();

  void on_turnLeft_pBtn_pressed();

  void on_turnLeft_pBtn_released();

  void on_turnRight_pBtn_pressed();

  void on_turnRight_pBtn_released();

private:
  Ui::moveUav *ui;
};
//}
#endif // MOVEUAV_HPP


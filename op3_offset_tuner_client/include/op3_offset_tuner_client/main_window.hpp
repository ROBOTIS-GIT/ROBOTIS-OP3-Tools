/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef OP3_OFFSET_TUNER_CLIENT_MAIN_WINDOW_H
#define OP3_OFFSET_TUNER_CLIENT_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <QtGui>
#include <QList>
#include <QSpinBox>
#include <QMainWindow>

#include <math.h>

#include "ui_main_window.h"
#include "qnode.hpp"

#endif
/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace op3_offset_tuner_client
{

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event);  // Overloaded function

 public Q_SLOTS:
  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();

  void on_save_button_clicked(bool check);
  void on_refresh_button_clicked(bool check);
  void on_inipose_button_clicked(bool checck);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically

  void updateJointOffsetSpinbox(op3_offset_tuner_msgs::JointOffsetPositionData msg);

  void changedSpinBoxValue(QString q_joint_name);
  void clickedTorqueCheckbox(QWidget *widget);
  void clickedAllTorqueOnButton(QObject *button_group);
  void clickedAllTorqueOffButton(QObject *button_group);

 private:
  void makeUI();
  void makeTabUI(QGroupBox *joint_widget, QGroupBox *torque_widget, QButtonGroup *button_group,
                 std::map<int, std::string> &offset_group);
  void publishTorqueMsgs(std::string &joint_name, bool torque_on);

  Ui::MainWindowDesign ui_;
  QNode qnode_;

  bool all_torque_on_;

  QButtonGroup *right_arm_button_group_;
  QButtonGroup *left_arm_button_group_;
  QButtonGroup *legs_button_group_;
  QButtonGroup *body_button_group_;
  std::vector<std::string> spinBox_list_;

  std::map<std::string, QList<QAbstractSpinBox *> > joint_spinbox_map_;
};

}  // namespace op3_offset_tuner_client

#endif // OP3_OFFSET_TUNER_CLIENT_MAIN_WINDOW_H

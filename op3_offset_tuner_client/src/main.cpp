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


/*****************************************************************************
** Includes
*****************************************************************************/

#include <QApplication>
#include "../include/op3_offset_tuner_client/main_window.hpp"
#include "rclcpp/rclcpp.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

    op3_offset_tuner_client::MainWindow w(argc, argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto spin_thread = std::thread([&executor]() {
        executor->spin();
    });

    int result = app.exec();

    // Shutdown ROS 2
    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();

    return result;
}

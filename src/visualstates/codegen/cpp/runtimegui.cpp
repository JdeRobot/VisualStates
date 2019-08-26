/*
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

*/

#include "../../../../include/visualstates/runtimegui.h"
#include <sstream>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


RunTimeGui::RunTimeGui() {
    rclcpp::Node::SharedPtr node;
    runningStatePublisher = node->create_publisher<std_msgs::msg::String>("/runtime_gui", 100);

    //runningStatePublisher = node->advertise<std_msgs::msg::String>("/runtime_gui", 100);
}

void RunTimeGui::emitRunningStateById(int id) {
    //std_msgs::msg::String runningStateMsg;
    auto runningStateMsg = std::make_shared<std_msgs::msg::String>();
    std::stringstream ss;
    ss << id;
    runningStateMsg->data = ss.str();
    runningStatePublisher->publish(runningStateMsg);
}

void RunTimeGui::emitLoadFromRoot() {
}

void RunTimeGui::emitActiveStateById(int id) {
}

/* Copyright (C) 2017 rerobots, Inc.
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
 */

#include <iostream>

#include "ros/ros.h"
#include "serial/serial.h"


int main( int argc, char **argv )
{
    ros::init( argc, argv, "bhand_diagnostics" );
    ros::NodeHandle nh;

    serial::Serial bhandcom( "/dev/ttyACM0", 115200,
                             serial::Timeout::simpleTimeout(1000) );
    if (bhandcom.isOpen()) {
        std::cout << "connected!" << std::endl;
    }

    ros::Rate rate( 1. );
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

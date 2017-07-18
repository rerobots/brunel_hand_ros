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
#include <string>
#include <cstdlib>

#include "ros/ros.h"
#include "serial/serial.h"

#include "brunel_hand_ros/FingerPose.h"


int main( int argc, char **argv )
{
    ros::init( argc, argv, "brunel_hand_proxy" );
    ros::NodeHandle nh;
    ros::Publisher posep;

    std::string devfile;
    nh.param( "hand_dev_file", devfile, std::string( "/dev/ttyACM0" ) );

    ROS_INFO( "Attempting to open %s", devfile.c_str() );
    serial::Serial bhandcom( devfile, 115200,
                             serial::Timeout::simpleTimeout(1000) );
    if (!bhandcom.isOpen()) {
        ROS_ERROR( "Failed to connect using %s", devfile.c_str() );
        return 1;
    }

    posep = nh.advertise<brunel_hand_ros::FingerPose>( "fpose", 10, true );

    ros::Rate rate( 100 );
    while (ros::ok()) {
        std::string line = bhandcom.readline();

        // If first item is not digit, then assume that this messages is not the
        // CSV finger data message type.
        if (line[0] != '0' && line[0] != '1' && line[0] != '2' && line[0] != '3'
            && line[0] != '4' && line[0] != '5' && line[0] != '6'
            && line[0] != '7' && line[0] != '8' && line[0] != '9')
            continue;

        brunel_hand_ros::FingerPose currentpose;
        currentpose.header.stamp = ros::Time::now();
        int thispos = -1;  // -1 => first search is with respect to position 0
        int nextpos = 0;
        std::string numstring;
        while ((nextpos = line.find( "," , thispos+1))
               != std::string::npos) {
            numstring = line.substr( thispos+1, nextpos-thispos-1 );
            thispos = nextpos;
            currentpose.f.push_back( std::atoi( numstring.c_str() ) );
        }
        numstring = line.substr( thispos+1 );
        if (numstring[0] != ' '
            && numstring[0] != '\r'
            &&  numstring[0] != '\n')
            currentpose.f.push_back( std::atoi( numstring.c_str() ) );

        posep.publish( currentpose );

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

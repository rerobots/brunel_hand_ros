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
#include <vector>
#include <string>
#include <cstdlib>

#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "serial/serial.h"

#include "brunel_hand_ros/FingerPose.h"
#include "brunel_hand_ros/RawCommand.h"
#include "brunel_hand_ros/HandPrimitive.h"


class BrunelHand
{
public:
    BrunelHand()
        : connected( false )
        {}

    BrunelHand( std::string devfile );

    std::vector<int> readFingersPose();

    /* Attempt (blocking) to get current firmware diagnostics.
       This method is atomic with respect to diagnostics data, i.e., either all
       known parameters are updated, or none at all. */
    void updateDiagnostics();

    /* Apply string directly as input to firmware connection.
       A newline character is appended to the given string. Of course, injection
       of multiple commands is feasible here, but the input is assumed to be
       trusted. Future versions may provide guards. */
    void rawCommand( std::string cmd );

    void callbackRawCommand( const brunel_hand_ros::RawCommand &rc );

    void callbackMotionPrimitive( const brunel_hand_ros::HandPrimitive &hp );

    /* Enable CSV mode.
       Diagnostics data are updated to verify whether CSV mode is enabled. */
    void useCSVmode();

    /* Enable motors if they are not already. */
    void enableMotors();

    bool getCSVmode() const;
    bool getMotorsEnabled() const;
    bool isConnected() const;

private:
    bool csvMode;
    bool motorsEnabled;
    bool connected;
    serial::Serial bhandcom;
    boost::mutex mtx_;
};

void BrunelHand::rawCommand( std::string cmd )
{
    mtx_.lock();
    bhandcom.flush();
    ROS_INFO( "SENDING: %s ", cmd.c_str() );
    bhandcom.write( cmd + "\n" );
    mtx_.unlock();
}

void BrunelHand::callbackRawCommand( const brunel_hand_ros::RawCommand &rc )
{
    rawCommand( rc.command );
}

void BrunelHand::callbackMotionPrimitive( const brunel_hand_ros::HandPrimitive &hp )
{
    switch (hp.primitive) {
    case 0: // Fist
        rawCommand( "G0" );
        break;

    case 1: // Palm
        rawCommand( "G1" );
        break;

    case 2: // Thumbs Up
        rawCommand( "G2" );
        break;

    case 3: // Point
        rawCommand( "G3" );
        break;

    case 4: // Pinch
        rawCommand( "G4" );
        break;

    case 5: // Tripod
        rawCommand( "G5" );
        break;

    case 6: // Finger Roll
        rawCommand( "G6" );
        break;

    case 7: // Thumb Roll
        rawCommand( "G7" );
        break;
    }
}

BrunelHand::BrunelHand( std::string devfile )
    : bhandcom( devfile, 115200,
                serial::Timeout::simpleTimeout(1000) ),
      connected( false )
{
    if (!bhandcom.isOpen()) {
        ROS_ERROR( "Failed to connect using %s", devfile.c_str() );
        return;
    }

    connected = true;
    ROS_INFO( "Connected via %s", devfile.c_str() );

    updateDiagnostics();
}

std::vector<int> BrunelHand::readFingersPose()
{
    std::vector<int> fpose;

    std::string line;
    mtx_.lock();
    // If first item is not digit, then assume that this messages is not the CSV
    // finger data message type.
    do {
        line = bhandcom.readline();
    } while (line[0] != '0' && line[0] != '1' && line[0] != '2'
             && line[0] != '3' && line[0] != '4' && line[0] != '5'
             && line[0] != '6' && line[0] != '7' && line[0] != '8'
             && line[0] != '9');
    mtx_.unlock();

    int thispos = -1;  // -1 => first search is with respect to position 0
    int nextpos = 0;
    std::string numstring;
    while ((nextpos = line.find( "," , thispos+1))
           != std::string::npos) {
        numstring = line.substr( thispos+1, nextpos-thispos-1 );
        thispos = nextpos;
        fpose.push_back( std::atoi( numstring.c_str() ) );
    }
    numstring = line.substr( thispos+1 );
    if (numstring[0] != ' '
        && numstring[0] != '\r'
        &&  numstring[0] != '\n')
        fpose.push_back( std::atoi( numstring.c_str() ) );
    return fpose;
}

bool BrunelHand::getCSVmode() const
{
    return csvMode;
}

bool BrunelHand::getMotorsEnabled() const
{
    return motorsEnabled;
}

void BrunelHand::useCSVmode()
{
    this->updateDiagnostics();
    if (csvMode)
        return;

    mtx_.lock();
    bhandcom.flush();
    bhandcom.write( "A4\n" );
    std::string line;
    do {
        line = bhandcom.readline();
    } while (line.substr( 0, 8 ) != "CSV mode");
    mtx_.unlock();
}

void BrunelHand::enableMotors()
{
    this->updateDiagnostics();
    if (motorsEnabled)
        return;

    mtx_.lock();
    bhandcom.flush();
    bhandcom.write( "A3\n" );
    std::string line;
    do {
        line = bhandcom.readline();
    } while (line.substr( 0, 14 ) != "Motors ENABLED");
    mtx_.unlock();
}

bool BrunelHand::isConnected() const
{
    return connected;
}

void BrunelHand::updateDiagnostics()
{
    int incoming_csvMode = -1;
    int incoming_motorsEnabled = -1;

    mtx_.lock();
    bhandcom.write( "#\n" );
    while (incoming_csvMode < 0 || incoming_motorsEnabled < 0) {
        std::string line = bhandcom.readline();
        if (line.substr( 0, 5 ) == "Mode:") {
            if (line.find( "CSV" ) != std::string::npos) {
                incoming_csvMode = 1;
            } else {
                incoming_csvMode = 0;
            }
        } else if (line.substr( 0, 7 ) == "Motors:") {
            if (line.find( "ENABLED" ) != std::string::npos) {
                incoming_motorsEnabled = 1;
            } else {
                incoming_motorsEnabled = 0;
            }
        }
    }
    mtx_.unlock();
    csvMode = incoming_csvMode ? true : false;
    motorsEnabled = incoming_motorsEnabled ? true : false;
}


int main( int argc, char **argv )
{
    ros::init( argc, argv, "brunel_hand_proxy" );
    ros::NodeHandle nh;
    ros::Publisher posep;
    ros::Subscriber subraw;
    ros::Subscriber submotprimitive;

    std::string devfile;
    nh.param( "hand_dev_file", devfile, std::string( "/dev/ttyACM0" ) );

    BrunelHand bhand( devfile );
    bhand.useCSVmode();
    bhand.enableMotors();

    posep = nh.advertise<brunel_hand_ros::FingerPose>( "fpose", 10, true );
    subraw = nh.subscribe( "/raw_input", 1,
                           &BrunelHand::callbackRawCommand, &bhand );
    submotprimitive = nh.subscribe( "/motion_primitive",
                                    1,
                                    &BrunelHand::callbackMotionPrimitive,
                                    &bhand );

    ros::Rate rate( 100 );
    while (ros::ok()) {
        brunel_hand_ros::FingerPose currentpose;
        currentpose.header.stamp = ros::Time::now();
        currentpose.f = bhand.readFingersPose();
        posep.publish( currentpose );

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

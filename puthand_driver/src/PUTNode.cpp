// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the PUT hand Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2018 Poznan University of Technology, Poznan, Poland
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Dominik Belter
 * \date    2018-05-19
 *
 */
//----------------------------------------------------------------------
// ROS includes.
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <string>

// Custom includes
#include "PUTNode.h"

#include <iostream>
#include <unistd.h>
#include <cstdlib>

/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

ros::Publisher channel_target_pub;
ros::Publisher ctrlPub;

PUTNode::PUTNode(const ros::NodeHandle& nh)
{
  //==========
  // Params
  //==========

  bool autostart, use_internal_logging;
  int reset_timeout;
  // Config that contains the log stream configuration without the file names
  std::string logging_config_file;

  // Parameters that depend on the hardware version of the hand.
  XmlRpc::XmlRpcValue dynamic_parameters;

  uint16_t manual_major_version = 0;
  int manual_major_version_int = 0;
  uint16_t manual_minor_version = 0;
  int manual_minor_version_int = 0;

  try
  {
    nh.param<bool>("autostart", autostart, false);
    nh.param<bool>("use_internal_logging", use_internal_logging, false);
    nh.param<std::string>("serial_device", serial_device_name_, "/dev/ttyUSB0");
    nh.param<int>("reset_timeout", reset_timeout, 5);
    nh.getParam("logging_config", logging_config_file);
    nh.param<std::string>("name_prefix", name_prefix, "left_hand");
    nh.param<int>("connect_retry_count", connect_retry_count, 3);
    simulation = true;
    nh.param<bool>("simulation", simulation, true);

    jointLimits.resize(13);
    for (int jointNo=0;jointNo<3;jointNo++){
        double limitMin, limitMax;
        nh.param<double>("Thumb_j"+std::to_string(jointNo+1)+"_min", limitMin, 0);
        nh.param<double>("Thumb_j"+std::to_string(jointNo+1)+"_max", limitMax, 0);
        jointLimits[jointNo] = std::make_pair(limitMin, limitMax);
    }
    for (int jointNo=0;jointNo<2;jointNo++){
        double limitMin, limitMax;
        nh.param<double>("Index_Finger_j"+std::to_string(jointNo+1)+"_min", limitMin, 0);
        nh.param<double>("Index_Finger_j"+std::to_string(jointNo+1)+"_max", limitMax, 0);
        jointLimits[jointNo+3] = std::make_pair(limitMin, limitMax);

        nh.param<double>("Middle_Finger_j"+std::to_string(jointNo+1)+"_min", limitMin, 0);
        nh.param<double>("Middle_Finger_j"+std::to_string(jointNo+1)+"_max", limitMax, 0);
        jointLimits[jointNo+5] = std::make_pair(limitMin, limitMax);
    }
    for (int jointNo=0;jointNo<3;jointNo++){
        double limitMin, limitMax;
        nh.param<double>("Ring_Finger_j"+std::to_string(jointNo+1)+"_min", limitMin, 0);
        nh.param<double>("Ring_Finger_j"+std::to_string(jointNo+1)+"_max", limitMax, 0);
        jointLimits[7+jointNo] = std::make_pair(limitMin, limitMax);
    }
    for (int jointNo=0;jointNo<3;jointNo++){
        double limitMin, limitMax;
        nh.param<double>("Pinky_Finger_j"+std::to_string(jointNo+1)+"_min", limitMin, 0);
        nh.param<double>("Pinky_Finger_j"+std::to_string(jointNo+1)+"_max", limitMax, 0);
        jointLimits[10+jointNo] = std::make_pair(limitMin, limitMax);
    }

    std::cout << "Limits:\n";
    for (const auto& l : jointLimits){
        std::cout << l.first << "->" << l.second << "\n";
    }
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
  }
  std::cout << "serial_device_name_ " << serial_device_name_ << "\n";
  // Tell the user what we are using
  ROS_INFO("Name prefix for this Hand was set to :%s", name_prefix.c_str());

  size_t channels = 13;
  // prepare the channel position message for later sending
  channel_pos_.name.resize(channels);
  channel_pos_.position.resize(channels, 0.0);
  channel_pos_.effort.resize(channels, 0.0);
  std::vector<std::string> chnames = {"Thumb_j1", "Thumb_j2", "Thumb_j3",
                                      "Index_Finger_j1", "Index_Finger_j2",
                                      "Middle_Finger_j1", "Middle_Finger_j2",
                                      "Ring_Finger_j1", "Ring_Finger_j2", "Ring_Finger_j3",
                                      "Pinky_Finger_j1", "Pinky_Finger_j2", "Pinky_Finger_j3"};
  for (size_t channel = 0; channel < channels; ++channel)
  {
    channel_pos_.name[channel] =
      name_prefix + "_" + chnames[channel];
  }
  channel_pos_.name.resize(channels);
  channel_pos_.position.resize(channels, 0.0);
  channel_pos_.effort.resize(channels, 0.0);
  for (size_t channel = 0; channel < channels; ++channel)
  {
    channel_pos_.name[channel] =
      name_prefix + "_" + chnames[channel];//
  }

  // Prepare the channel current message for later sending
  channel_currents.data.clear();
  channel_currents.layout.data_offset = 0;
  std_msgs::MultiArrayDimension dim;
  dim.label  = "channel currents";
  dim.size   = channels;
  dim.stride = 0;
  channel_currents.layout.dim.push_back(dim);

  if (!simulation){
      std::cout << "initialize serial port\n";
      initializeSerialPort();
      std::cout << "initialize serial port done\n";
      modeHand = EXTERNAL;
      std::string enableStatusPayload = liderHandParser.PrepareDataEnableStatusUpdate();
      std::cout << "initialize device\n";
      std::cout << "enableStatusPayload " << enableStatusPayload << "\n";
      serialPort.write(enableStatusPayload.c_str(), enableStatusPayload.length());
      std::cout << "initialize device finished\n";
  }
}

PUTNode::~PUTNode()
{
  // Tell the driver to close connections
  //fm_->disconnect();
}

void PUTNode::initializeSerialPort(){
    isSerialOK = false;
    std::string command = "stty -F " + serial_device_name_ + " 460800";
    system(command.c_str());
    serialPort.Open( serial_device_name_ );
    if ( ! serialPort.good() ) {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl ;
        return;
    }
    // Set the baud rate of the serial port.
    serialPort.SetBaudRate( LibSerial::SerialStreamBuf::BAUD_460800 ) ;
    if ( ! serialPort.good() ){
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        return;
    }
    // Set the number of data bits.
    serialPort.SetCharSize( LibSerial::SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serialPort.good() ){
        std::cerr << "Error: Could not set the character size." << std::endl ;
        return;
    }
    // Disable parity.
    serialPort.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serialPort.good() ) {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        return;
    }
    // Set the number of stop bits.
    serialPort.SetNumOfStopBits( 1 ) ;
    if ( ! serialPort.good() ) {
        std::cerr << "Error: Could not set the number of stop bits." << std::endl ;
        return;
    }
    // Turn off hardware flow control.
    serialPort.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serialPort.good() ) {
        std::cerr << "Error: Could not use hardware flow control." << std::endl ;
        return;
    }
    system(command.c_str());
    isSerialOK = true;
}

/// read from serial port
char PUTNode::readBlocking(LibSerial::SerialStream& serial_port, int timeout) {
     char byteRec;
     serial_port.get(byteRec);
     return byteRec;
}

/// read line
int PUTNode::rxstring(std::string& line){
    line="";
    char temp=0;
    while(temp!='\n'){
        serialPort.get(temp);
        line.append(&temp);
    }
    return 0;
}

// Callback function for changing parameters dynamically
void PUTNode::dynamic_reconfigure_callback(puthand_controller::puthandConfig& config, uint32_t level)
{
  serial_device_name_ = config.serial_device;
}

// Callback function for connecting to SCHUNK five finger hand
void PUTNode::connectCallback(const std_msgs::Empty&)
{
//    std::cout << "connectCallback\n";
    ROS_ERROR(
      "Could not connect to PUT hand with serial device %s, and retry count %i",
      serial_device_name_.c_str(),
      connect_retry_count);
}

// Callback function to reset/home channels of SCHUNK five finger hand
void PUTNode::resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
    std::cout << "reset channelCallback\n";
}

// Callback function to enable channels of SCHUNK five finger hand
void PUTNode::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
    std::cout << "enable channel Callback\n";
}

/// check error code and print message
void PUTNode::checkError(const LiderHand::CurrentError_Type& error) const{
    if(error != LiderHand::ERROR_OK) {//system error handling (also indicated by LED blinking)
        if(error & LiderHand::ERROR_RS485_TIMEOUT) {
//            std::cout << "LiderHand RS485 TIMEOUT ERROR" << std::endl;
        }
        if(error & LiderHand::ERROR_RS485_CRC) {
            std::cout << "LiderHand RS485 CRC ERROR" << std::endl;
        }
        if(error & LiderHand::ERROR_MOTOR_FAULT) {
            std::cout << "LiderHand MOTOR ERROR" << std::endl;
        }
        if(error & LiderHand::ERROR_FT232_CRC) {
            //                    std::cout << "LiderHand PC CRC ERROR" << std::endl;
        }
    }
}

/// set reference position
void PUTNode::setReferencePosition(size_t jointNo, double position){
    uint16_t maxRange = std::numeric_limits<uint16_t>::max();
    uint16_t refPosition=32000;
    if (jointNo==0)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    else if (jointNo==1)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position+(maxRange/2);
    else if (jointNo==2)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    else if (jointNo==3)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *(position-jointLimits[jointNo].first);
    else if (jointNo==4)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    else if (jointNo==5)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *(position-jointLimits[jointNo].first);
    else if (jointNo==6)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    else if (jointNo==7)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    else if (jointNo==8)
        refPosition = (maxRange/(jointLimits[jointNo].second-jointLimits[jointNo].first)) *position;
    liderHandParser.SetPosition(jointNo, refPosition); //set position of each drive here, range 1-65535

    std::string InternalPayload = liderHandParser.PrepareDataInternalRegMode();
    serialPort.write(InternalPayload.c_str(), InternalPayload.length());
}

// Callback function for setting channel target positions to PUT hand
void PUTNode::jointStateCallback(const sensor_msgs::JointStateConstPtr& input){
    if (!simulation&&isSerialOK){
        std::cout << "input.get()->position " << input.get()->position.size() << "\n";
        std::string line;
        rxstring(line);
        if(liderHandParser.ParseDataFromLiderHand(line) == LiderHand::SUCCESS){
            uint8_t count = 9;//liderHandParser.GetMotorDriverCount(); //acces driver count
            LiderHand::CurrentError_Type error = liderHandParser.GetCurrentError();

            checkError(error);
            for(int i=0; i<count; i++)
            {
                LiderHand::MotorDriverOperation_Type op = liderHandParser.GetMotorDriverOperation(i);

                if(op == LiderHand::Operation_Fault) //handle particula driver error
                {
                    std::cout << "Motor " << i << " faulty" << std::endl;
                }
                if (i<5)
                    setReferencePosition(i,input.get()->position[i]);
                else if (i>4&&i<7)
                    setReferencePosition(i,input.get()->position[i+1]);
                else if (i==7){
                    setReferencePosition(i,input.get()->position[i+2]);
                }
                else if (i==8){
                    setReferencePosition(i,input.get()->position[i+4]);
                }
            }
        }
        else
            std::cout << "Data from device incorrect\n";
    }
}


sensor_msgs::JointState PUTNode::getChannelFeedback()
{
    if (!simulation&&isSerialOK){
        std::string line;
        rxstring(line);
        if(liderHandParser.ParseDataFromLiderHand(line) == LiderHand::SUCCESS){
            uint8_t count = 13;// liderHandParser.GetMotorDriverCount(); //acces driver count
            LiderHand::CurrentError_Type error = liderHandParser.GetCurrentError();
            checkError(error);
            for(int i=0; i<count; i++)
            {
                LiderHand::MotorDriverOperation_Type op = liderHandParser.GetMotorDriverOperation(i);
                if(op == LiderHand::Operation_Fault) //handle particula driver error
                {
                    std::cout << "Motor " << i << " faulty" << std::endl;
                    break;
                }
                //                uint16_t pwm = liderHandParser.GetPWM(i);
                //                uint16_t cur = liderHandParser.GetCurrent(i);
                uint16_t posCur;
                if (i>6&&i<10){
                    posCur = liderHandParser.GetPositonCurrent(7,i-7);
                }
                else if (i>9){
                    posCur = liderHandParser.GetPositonCurrent(8,i-10);
                }
                else {
                    posCur = liderHandParser.GetPositonCurrent(i,0);
                }
                double scaleFactor = double(posCur)/double(std::numeric_limits<uint16_t>::max());
                channel_pos_.position[i] = (jointLimits[i].second-jointLimits[i].first)*scaleFactor + jointLimits[i].first;
            }
        }
        else
            std::cout << "Data from device incorrect\n";
    }

    channel_pos_.header.stamp = ros::Time::now();
    return channel_pos_;
}

std_msgs::Float64MultiArray PUTNode::getChannelCurrents()
{
  return channel_currents;
}

/// received string order
void PUTNode::orderCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << "order callback\n";
    control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
    for (const auto& name : channel_pos_.name){
        ctrlMsg.goal.trajectory.joint_names.push_back(name);
    }

    ctrlMsg.goal.trajectory.points.resize(1);
    ctrlMsg.goal.trajectory.points[0].positions.resize(channel_pos_.name.size());
    if (msg->data.compare("moveInit")==0){//2.1, 0.6, 0.7, 0.3, 1.2, 0.3, 1.2, 0.1, 0.1, 0.1
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({2.1, 0.6, 0.7, 0.3, 1.2, 0.3, 1.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({2.1, 0.6, 0.7, 0.3, 1.2, 0.3, 1.2, 0.1, 0.1});
    }
    else if (msg->data.compare("movePinch")==0){//0.4, 0.2, 1.25, 1.1, 1.05, 0.3, 1.65, 0.1, 0.1
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.4, 0.2, 1.25, 1.1, 1.05, 0.3, 1.65, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.4, 0.2, 1.25, 1.1, 1.05, 0.3, 1.65, 0.1, 0.1});
        std::cout << "movePinch\n";
    }
    else if (msg->data.compare("almostPinch")==0){//0.24, 0.67, 0.70, 1.02, 0.97, 0.25, 1.20, 0.0, 0.0
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.70, 1.02, 0.97, 0.25, 1.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.70, 1.02, 0.97, 0.25, 1.20, 0.0});
        std::cout << "almostPinch\n";
    }
    else if (msg->data.compare("pinchCable")==0){//0.24, 0.67, 0.70, 1.02, 0.97, 0.25, 1.20, 0.0, 0.0
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.70, 1.20, 0.97, 0.25, 1.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.70, 1.20, 0.97, 0.25, 1.20, 0.0});
        std::cout << "almostPinch\n";
    }
    else if (msg->data.compare("closeHand")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.4, 0.2, 1.25, 1.1, 1.05, 1.1, 1.05, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.4, 0.2, 1.25, 1.1, 1.05, 1.1, 1.05, 1.57, 1.57});
        std::cout << "close hand\n";
    }
    else if (msg->data.compare("grabGreenRubber")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 1.14, 1.05, 1.14, 1.05, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 1.14, 1.05, 1.14, 1.05, 0.2, 0.2});
    }
    else if (msg->data.compare("grabGreenRubberMore")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.6, 0.99, 1.14, 1.05, 1.14, 1.05, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.6, 0.99, 1.14, 1.05, 1.14, 1.05, 0.2, 0.2});
    }
    else if (msg->data.compare("grab")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.65, 0.88, 1.24, 1.05, 1.14, 1.05, 0.77, 0.77, 0.77, 0.77, 0.77, 0.77});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.65, 0.77, 1.1, 0.98, 1.14, 1.05, 0.77, 0.77});
    }
    else if (msg->data.compare("explore")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.6, 0.99, 1.24, 1.05, 1.14, 1.05, 0.77, 0.77, 0.77, 0.77, 0.77, 0.77});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.65, 0.7, 1.14, 0.95, 1.14, 1.05, 0.77, 0.77});

    }
    else if (msg->data.compare("prepare")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.6, 0.99, 1.24, 1.05, 1.14, 1.05, 0.77, 0.77, 0.77, 0.77, 0.77, 0.77});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.8, 0.4, 0.8, 1.5, 0.44, 1.55, 0.2, 0.2});
    }
    else if (msg->data.compare("grabSquashBall")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 0.77, 0.95, 0.77, 0.95, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 0.77, 0.95, 0.77, 0.95, 0.2, 0.2});
    }
    else if (msg->data.compare("grabSquashBallMore")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 0.97, 0.95, 0.97, 0.95, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({0.24, 0.67, 0.88, 0.97, 0.95, 0.97, 0.95, 0.2, 0.2});
    }
    else if (msg->data.compare("openHand")==0){
        if (useSimul())
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({2.1, 0.6, 0.7, 0.25, 1.67, 0.25, 1.67, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1});
        else
            ctrlMsg.goal.trajectory.points[0].positions = std::vector<double>({2.1, 0.6, 0.7, 0.25, 1.67, 0.25, 1.67, 0.0, 0.0});
        std::cout << "open Hand\n";
    }
    else
        return;
    // Velocities
    ctrlMsg.goal.trajectory.points[0].velocities.resize(channel_pos_.name.size());
    for (size_t jointNo=0; jointNo<channel_pos_.name.size();jointNo++)
        ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
    ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    ctrlPub.publish(ctrlMsg);
}

void PUTNode::execute(const control_msgs::FollowJointTrajectoryGoal::ConstPtr goal, ActionServer* as) {// Note: "Action" is not appended to DoDishes here
    std::cout << "start action\n";
    if (!simulation){
        double maxError=0.5;//rad
        //for (const auto& point : goal.get()->trajectory.points){
        const auto point = goal.get()->trajectory.points.back();//we don't try to execute trajectory, we send the last position only
        {
            size_t jointNo = 0;
            for (const auto pos : point.positions){
                auto it = std::find(channel_pos_.name.begin(), channel_pos_.name.end(), goal.get()->trajectory.joint_names[jointNo]);
                if (it == channel_pos_.name.end()){
                    std::cout << "Incorrect joint name.\n";
                }
                else {
                    auto index = std::distance(channel_pos_.name.begin(), it);
                    setReferencePosition(index, pos);
                }
//                setReferencePosition(jointNo, pos);
                jointNo++;
            }
            std::cout << "send pose\n";
            bool pointExecuted(false);
            int maxTrials = 100;
            int trialNo = 0;
            jointNo=0;
            while (!pointExecuted){
                jointNo=0;
                bool isBiggerError(false);
                for (const auto pos : point.positions){//compute joint errors
                    double error=100;
                    auto it = std::find(channel_pos_.name.begin(), channel_pos_.name.end(), goal.get()->trajectory.joint_names[jointNo]);
                    if (it == channel_pos_.name.end()){
                        std::cout << "Incorrect joint name.\n";
                    }
                    else {
                        auto index = std::distance(channel_pos_.name.begin(), it);
                        setReferencePosition(index, pos);
                        error = channel_pos_.position[index] - pos;
                    }
                    if (fabs(error)>maxError){
                        isBiggerError = true;
                    }
                    jointNo++;
                }
                if (!isBiggerError)
                    pointExecuted = true;
                else
                    usleep(40000);//has to wait because the puthand does not like to many orders
                trialNo++;
                if (trialNo > maxTrials)
                    break;
            }
            usleep(800);
        }
        std::cout << "executed\n";
        as->setSucceeded();
    }
}

bool PUTNode::useSimul(void){
    return simulation;
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  //==========
  // ROS
  //==========

  // Set up ROS.
  ros::init(argc, argv, "puthand_controller");
  // Private NH for general params
  ros::NodeHandle nh("~");


  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(50);
  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  PUTNode put_node(nh);
  //==========
  // Dynamic Reconfigure
  //==========

  dynamic_reconfigure::Server<puthand_controller::puthandConfig> server;
  dynamic_reconfigure::Server<puthand_controller::puthandConfig>::CallbackType f;

  f = boost::bind(&PUTNode::dynamic_reconfigure_callback, &put_node, _1, _2);
  server.setCallback(f);
  //==========
  // Callbacks
  //==========

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, &PUTNode::connectCallback, &put_node);
  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub =
    nh.subscribe("reset_channel", 1, &PUTNode::resetChannelCallback, &put_node);
  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub =
    nh.subscribe("enable_channel", 1, &PUTNode::enableChannelCallback, &put_node);
  // Subscribe joint state topic

  ros::Subscriber channel_target_sub;
  if (!put_node.useSimul())
        channel_target_sub = nh.subscribe<sensor_msgs::JointState>("channel_targets",
                                          1,
                                          &PUTNode::jointStateCallback,
                                          &put_node,
                                          ros::TransportHints().tcpNoDelay());
  channel_target_pub =
          nh.advertise<sensor_msgs::JointState>("channel_targets",
                                                1);

  // Publish current channel positions
  ros::Publisher channel_pos_pub;
  if (!put_node.useSimul())
    channel_pos_pub = nh.advertise<sensor_msgs::JointState>("channel_feedback", 1);
  // Additionally publish just the current values of the motors
  ros::Publisher channel_current_pub;
  if (!put_node.useSimul())
    channel_current_pub = nh.advertise<std_msgs::Float64MultiArray>("channel_currents", 1);

  //std::unique_ptr<ActionServer> actionServer;
  
  std::cout << "ssss\n";
  if (!put_node.useSimul()){ //uncomment this if you use simulation
      std::cout << "start action server\n";
      //actionServer.reset(new ActionServer(nh, "follow_joint_trajectory", boost::bind(&PUTNode::execute, &put_node, _1, actionServer.get()), false));
      ActionServer actionServer(nh, "follow_joint_trajectory", boost::bind(&PUTNode::execute, &put_node, _1, &actionServer), false);
      
      actionServer.start();
      std::cout << "action server started\n";
  }

  // subsribe topic
  ros::Subscriber subOrders = nh.subscribe("/puthand_controller/order", 1, &PUTNode::orderCallback, &put_node);//subscribe orders topic

//  if (!put_node.useSimul())
  ctrlPub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/puthand/puthand_controller/follow_joint_trajectory/goal", 1000);
  //==========
  // Messaging
  //==========

  // Main loop.
  while (nh.ok())
  {
    // get the current positions of all joints and publish them
    if (!put_node.useSimul()){
        channel_pos_pub.publish(put_node.getChannelFeedback());
        channel_current_pub.publish(put_node.getChannelCurrents());
    }

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Dominik Belter <dominik.belter@put.poznan.pl>
 * \date    2018-06-15
 *
 */
//----------------------------------------------------------------------

#ifndef PUTHAND_NODE_H
#define PUTHAND_NODE_H

// Messages
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <puthand_driver/puthandConfig.h>

#include <boost/shared_ptr.hpp>
#include <thread>         // std::thread
#include <mutex>

#include <SerialStream.h>
#include <SerialPort.h>
#include <../liderhand-pc-commprotocol/liderhand.h>

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;

class PUTNode{

public:
  //!
  //! \brief PUTNode constructs a new node object that handles most of the functionality
  //! \param nh ROS Nodehandle
  //!
  PUTNode(const ros::NodeHandle &nh);
  //! Default destructor
  ~PUTNode();

  typedef enum
  {
      IDLE,
      INTERNAL,
      EXTERNAL
  }ModeType;

  /// initialize serial port
  void initializeSerialPort();

  //! Dynamic reconfigure callback to update changing parameters
  void dynamic_reconfigure_callback(puthand_controller::puthandConfig &config, uint32_t level);

  //! Callback function for connecting to PUThand five finger hand
  void connectCallback(const std_msgs::Empty&);

  //! Callback function to reset/home channels of PUThand five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of PUThand five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function for setting channel target positions to PUThand five finger hand
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& input);

  /// FollowJointTrajectory execution
  void execute(const control_msgs::FollowJointTrajectoryGoal::ConstPtr goal, ActionServer* as);

  /// read from serial port
  char readBlocking(LibSerial::SerialStream& serial_port, int timeout);

  /// read line
  int rxstring(std::string& line);

  //!
  //! \brief PUTNode::getChannelFeedback Gets the latest received positions and efforts from the driver
  //! \returns The current joint states (Position and Efforts)
  //!
  sensor_msgs::JointState getChannelFeedback();

  //!
  //! \brief getChannelCurrents Returns the current values of the channels as raw output
  //! \return Array containing the current values of the channels in mA
  //!
  std_msgs::Float64MultiArray getChannelCurrents();

  /// received string order
  void orderCallback(const std_msgs::String::ConstPtr& msg);

  bool useSimul(void);

private:
  //! Serial device to use for communication with hardware
  std::string serial_device_name_;

  /// use simulation
  bool simulation;

  //! Number of times the connect routine tries to connect in case that we receive at least one package
  int connect_retry_count;

  //! Prefix for the driver to identify joint names if the Driver should expext "left_hand_Pinky" than the prefix is left_hand
  std::string name_prefix;

  //! joint state message template
  sensor_msgs::JointState channel_pos_;

  //! Current Value message template
  std_msgs::Float64MultiArray channel_currents;

  /// serial port
  LibSerial::SerialStream serialPort;

  /// is serial ok
  bool isSerialOK;

  /// serial parser
  LiderHand liderHandParser;

  /// hand operation mode
  ModeType modeHand;

  /// joint limits
  std::vector<std::pair<double,double>> jointLimits;

  /// check error code and print message
  void checkError(const LiderHand::CurrentError_Type& error) const;

  /// set reference position
  void setReferencePosition(size_t jointNo, double position);
};

#endif //PUTHAND_NODE_H

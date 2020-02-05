#include "ros/ros.h"
#include "ros/package.h"
#include "ros/this_node.h"
#include <iostream>

#include "control_msgs/JointTrajectoryControllerState.h" //read state of the arm
#include "control_msgs/FollowJointTrajectoryAction.h" //set goal positions
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <stdio.h>

using namespace std;

ros::Publisher ctrlPub;//control the robot
std::vector<double> initPos = {1.1,0.6,0.7, 0.6,1.0, 0.6,1.0, 0.8,0.8,0.8, 0.8,0.8,0.8};
bool isHandReady = false;
size_t jointsNo = initPos.size();

/// read state of the robot
void puthandStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    if (msg->actual.positions.size()>0){
        for (const auto& jn : msg->joint_names)
            std::cout << jn << "\n";
        isHandReady = true;
    }
}

/// set initial configuration of the hand in the simulation and quit
int main(int argc, char **argv) {
    //initialize node
    ros::init(argc, argv, "puthand_set_init_conf");
    // node handler
    ros::NodeHandle n;
    // subsribe topic
   ros::Subscriber subRobot = n.subscribe("/puthand/puthand_controller/state", 1, puthandStateCallback);//simulator

    ctrlPub = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/puthand/puthand_controller/follow_joint_trajectory/goal", 1000);

    //ros loop
//    ros::spin();
    while (ros::ok()) {
        if (isHandReady){
            control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Thumb_j1");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Thumb_j2");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Thumb_j3");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Index_Finger_j1");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Index_Finger_j2");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Middle_Finger_j1");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Middle_Finger_j2");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Ring_Finger_j1");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Ring_Finger_j2");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Ring_Finger_j3");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Pinky_Finger_j1");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Pinky_Finger_j2");
            ctrlMsg.goal.trajectory.joint_names.push_back("left_hand_Pinky_Finger_j3");

            ctrlMsg.goal.trajectory.points.resize(1);
            ctrlMsg.goal.trajectory.points[0].positions.resize(jointsNo);
            ctrlMsg.goal.trajectory.points[0].positions = initPos;

            // Velocities
            ctrlMsg.goal.trajectory.points[0].velocities.resize(jointsNo);
            for (size_t jointNo=0; jointNo<jointsNo;jointNo++)
                ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;
            ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

            ctrlPub.publish(ctrlMsg);
            return 0;
        }
        ros::spinOnce();
    }
    return 0;
}

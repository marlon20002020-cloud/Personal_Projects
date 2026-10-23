/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

extern float gaitPeriod;
extern float forward_velocity;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);

void forward_walk();
void sendMoveCommand();

struct jointAngles {
	float q1_lf[24]; // left front
	float q1_rf[24]; // right front
	float q1_lr[24]; // left rear
	float q1_rr[24]; // right rear;
	float q2_lf[24];
	float q2_rf[24];
	float q2_lr[24];
	float q2_rr[24];
};

extern jointAngles Walk;
extern jointAngles Trot;
extern jointAngles Gallop;


}

#endif

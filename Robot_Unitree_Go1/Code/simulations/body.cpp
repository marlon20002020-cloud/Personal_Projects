/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace unitree_model {

ros::Publisher servo_pub[12];
ros::Publisher highState_pub;
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// Define the joint angles for the walk gait
float gaitPeriod = 1.0; // sec
float forward_velocity = 0.5; // m per s

// Define the gait structures to reference
jointAngles Walk = {
	{ 0.969, 1.319, 1.759, 0.939, 0.138, -0.075, -0.018, 0.078, 0.168, 0.274, 0.354, 0.431, 0.522, 0.590, 0.655, 0.729, 0.782, 0.830, 0.873, 0.916, 0.943, 0.962, 0.972, 0.969}, // q1_lf
	{ 0.852, 0.891, 0.930, 0.953, 0.968, 0.972, 1.040, 1.437, 1.759, 0.651, 0.046, -0.075, 0.007, 0.101, 0.189, 0.294, 0.374, 0.449, 0.522, 0.607, 0.670, 0.729, 0.795, 0.852}, // q1_rf
	{ -0.043, 0.055, 0.168, 0.253, 0.334, 0.431, 0.504, 0.573, 0.639, 0.714, 0.769, 0.819, 0.873, 0.908, 0.937, 0.962, 0.972, 0.971, 1.117, 1.654, 1.553, 0.430, -0.054, -0.043}, // q1_lr
	{ 0.486, 0.556, 0.639, 0.700, 0.756, 0.819, 0.863, 0.900, 0.930, 0.958, 0.970, 0.972, 1.117, 1.552, 1.712, 0.430, -0.015, -0.062, 0.007, 0.124, 0.211, 0.294, 0.393, 0.486}, // q1_rr
	{ -0.856, -1.195, -2.123, -2.429, -1.724, -0.971, -0.883, -0.979, -1.059, -1.140, -1.192, -1.234, -1.271, -1.290, -1.299, -1.298, -1.286, -1.265, -1.234, -1.180, -1.126, -1.059, -0.957, -0.856}, // q2_lf
	{ -1.250, -1.214, -1.155, -1.094, -1.021, -0.909, -0.908, -1.352, -2.123, -2.299, -1.529, -0.971, -0.909, -1.001, -1.077, -1.155, -1.204, -1.242, -1.271, -1.293, -1.300, -1.298, -1.282, -1.250}, // q2_rf
	{ -0.856, -0.957, -1.059, -1.126, -1.180, -1.234, -1.265, -1.286, -1.298, -1.299, -1.290, -1.271, -1.234, -1.192, -1.140, -1.059, -0.979, -0.883, -0.971, -1.724, -2.429, -2.123, -1.195, -0.856}, // q2_lr
	{ -1.258, -1.282, -1.298, -1.300, -1.293, -1.271, -1.242, -1.204, -1.155, -1.077, -1.001, -0.909, -0.971, -1.529, -2.299, -2.123, -1.352, -0.908, -0.909, -1.021, -1.094, -1.155, -1.214, -1.258} // q2_rr
};

jointAngles Trot = {
	{ 0.988, 1.148, 1.361, 1.594, 1.746, 1.632, 1.064, 0.520, 0.214, 0.063, 0.011, 0.024, 0.082, 0.218, 0.344, 0.461, 0.570, 0.670, 0.759, 0.837, 0.900, 0.949, 0.979, 0.988}, // q1_lf
	{ 0.034, 0.174, 0.303, 0.423, 0.535, 0.638, 0.731, 0.812, 0.881, 0.934, 0.971, 0.988, 1.039, 1.212, 1.440, 1.660, 1.751, 1.485, 0.852, 0.397, 0.149, 0.037, 0.010, 0.034}, // q1_rf
	{ 0.034, 0.174, 0.303, 0.423, 0.535, 0.638, 0.731, 0.812, 0.881, 0.934, 0.971, 0.988, 1.039, 1.212, 1.440, 1.660, 1.751, 1.485, 0.852, 0.397, 0.149, 0.037, 0.010, 0.034}, // q1_lr
	{ 0.988, 1.148, 1.361, 1.594, 1.746, 1.632, 1.064, 0.520, 0.214, 0.063, 0.011, 0.024, 0.082, 0.218, 0.344, 0.461, 0.570, 0.670, 0.759, 0.837, 0.900, 0.949, 0.979, 0.988}, // q1_rr
	{ -0.920, -1.056, -1.297, -1.642, -2.045, -2.388, -2.454, -2.174, -1.774, -1.403, -1.121, -0.962, -0.968, -1.087, -1.177, -1.240, -1.281, -1.299, -1.296, -1.270, -1.222, -1.150, -1.051, -0.920}, // q2_lf
	{ -0.920, -1.051, -1.150, -1.222, -1.270, -1.296, -1.299, -1.281, -1.240, -1.177, -1.087, -0.968, -0.962, -1.121, -1.403, -1.774, -2.174, -2.454, -2.388, -2.045, -1.642, -1.297, -1.056, -0.920}, // q2_rf
	{ -0.920, -1.051, -1.150, -1.222, -1.270, -1.296, -1.299, -1.281, -1.240, -1.177, -1.087, -0.968, -0.962, -1.121, -1.403, -1.774, -2.174, -2.454, -2.388, -2.045, -1.642, -1.297, -1.056, -0.920}, // q2_lr
	{ -0.920, -1.056, -1.297, -1.642, -2.045, -2.388, -2.454, -2.174, -1.774, -1.403, -1.121, -0.962, -0.968, -1.087, -1.177, -1.240, -1.281, -1.299, -1.296, -1.270, -1.222, -1.150, -1.051, -0.920} // q2_rr
};

jointAngles Gallop = {
	{ 0.971, 1.069, 1.177, 1.308, 1.450, 1.583, 1.671, 1.708, 1.621, 1.349, 0.968, 0.643, 0.414, 0.258, 0.160, 0.105, 0.079, 0.076, 0.084, 0.223, 0.501, 0.730, 0.894, 0.971}, // q1_lf
	{ 0.967, 1.040, 1.144, 1.269, 1.410, 1.548, 1.646, 1.707, 1.663, 1.445, 1.077, 0.725, 0.471, 0.296, 0.182, 0.117, 0.084, 0.075, 0.081, 0.134, 0.426, 0.670, 0.855, 0.967}, // q1_rf
	{ 0.568, 0.363, 0.224, 0.140, 0.095, 0.076, 0.077, 0.087, 0.307, 0.572, 0.784, 0.927, 0.985, 1.083, 1.195, 1.328, 1.470, 1.600, 1.681, 1.705, 1.594, 1.297, 0.915, 0.568}, // q1_lr
	{ 0.441, 0.276, 0.170, 0.111, 0.081, 0.075, 0.082, 0.179, 0.464, 0.700, 0.876, 0.967, 1.040, 1.144, 1.269, 1.410, 1.548, 1.659, 1.707, 1.663, 1.445, 1.077, 0.725, 0.441}, // q1_rr
	{ -0.988, -1.076, -1.183, -1.333, -1.521, -1.739, -1.942, -2.176, -2.372, -2.475, -2.432, -2.268, -2.044, -1.805, -1.580, -1.384, -1.221, -1.103, -1.026, -1.103, -1.263, -1.298, -1.211, -0.988}, // q2_lf
	{ -1.030, -1.050, -1.148, -1.286, -1.464, -1.674, -1.873, -2.111, -2.323, -2.459, -2.459, -2.323, -2.111, -1.873, -1.642, -1.437, -1.264, -1.133, -1.050, -1.030, -1.231, -1.300, -1.249, -1.030}, // q2_rf
	{ -2.208, -1.976, -1.739, -1.521, -1.333, -1.183, -1.089, -1.001, -1.163, -1.286, -1.286, -1.163, -1.001, -1.089, -1.202, -1.358, -1.550, -1.772, -1.976, -2.208, -2.394, -2.478, -2.414, -2.208}, // q2_lr
	{ -2.078, -1.839, -1.611, -1.410, -1.242, -1.117, -1.038, -1.068, -1.249, -1.300, -1.231, -1.030, -1.050, -1.148, -1.286, -1.464, -1.674, -1.908, -2.111, -2.323, -2.459, -2.459, -2.323, -2.078} // q2_rr
};

// Joint angles are sent in this order for each leg: [hip, thigh, calf]
// Joint angles are sent in those order of legs: [FR, FL, RR, RL]
// We don't want to move hip joints - set those to 0.0.
// To access a single q value: float q = Walk.q1_lf[0]
// A joint angle position command is sent as a 12-element array.
// q1 joint angles must be between -0.663 and 2.967 rads
// q2 joint angles must be between -2.723 and -0.838 rads

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000); // 1 ms, this sets the frequency 
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent;
        }
        sendServoCmd();
    }
}

// New funcs created for gait forward movement

// Timing math
int num_qs_in_gait = sizeof(Walk.q1_lf) / sizeof(Walk.q1_lf[0]);
float time_per_q_us_f = (gaitPeriod * 1000000.0) / num_qs_in_gait;
int time_per_q_us = static_cast<int>(std::round(time_per_q_us_f));

// Function to send each set of joint angles and set timing
void sendMoveCommand() {
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(time_per_q_us); // Set the period between q's, so that we can set the velocity
}

// function to make walk work
void forward_walk() {
    // Cycle through the whole gait sequence
    for (int i = 0; i < num_qs_in_gait; i++) {
        // Load the motorCmd structure with the joint angles to send
        lowCmd.motorCmd[0].q = 0.0;                 // FR Hip
        lowCmd.motorCmd[1].q = Walk.q1_rf[i];       // FR Thigh
        lowCmd.motorCmd[2].q = Walk.q2_rf[i];       // FR Calf
        lowCmd.motorCmd[3].q = 0.0;                 // FL Hip
        lowCmd.motorCmd[4].q = Walk.q1_lf[i];       // FL Thigh
        lowCmd.motorCmd[5].q = Walk.q2_lf[i];       // FL Calf
        lowCmd.motorCmd[6].q = 0.0;                 // RR Hip
        lowCmd.motorCmd[7].q = Walk.q1_rr[i];       // RR Thigh
        lowCmd.motorCmd[8].q = Walk.q2_rr[i];       // RR Calf
        lowCmd.motorCmd[9].q = 0.0;                 // RL Hip
        lowCmd.motorCmd[10].q = Walk.q1_lr[i];      // RL Thigh
        lowCmd.motorCmd[11].q = Walk.q2_lr[i];      // RL Calf
        // Now send the commands to the robot
        sendMoveCommand();
    }
}


}

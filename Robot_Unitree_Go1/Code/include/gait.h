#ifndef GAIT_H
#define GAIT_H
#include "spline.h" // #pragma once
#include <array>
#include <cmath>   // For sin, cos, atan2
#include <vector>

// Macros
#define NUM_INTERVALS		100
#define LINK2_LENGTH		0.213
#define LINK3_LENGTH		0.212272
#define THIGH_JOINT_MIN		-0.6632  // rads, in the Unitree frame
#define THIGH_JOINT_MAX		2.9671
#define CALF_JOINT_MIN		-2.7227
#define CALF_JOINT_MAX		-0.8378
#define FORWARD_VEL			0.5
#define GAIT_PERIOD			1

// Aliases for long types
using GaitPath = std::array<std::array<double, NUM_INTERVALS>, 2>; // a 2D array equivalent to x[2][NUM_INTERVALS]
using ThetaTraj = std::array<std::array<double, NUM_INTERVALS>, 3>; // A 2D array for storing thetas - th[3][NUM_INTERVALS]
using Vector = std::vector<double>;

// Structs
struct LegJointAngles {
	std::array<double, NUM_INTERVALS> q1; // hip joint
	std::array<double, NUM_INTERVALS> q2; // thigh joint
	std::array<double, NUM_INTERVALS> q3; // calf joint 
};

struct GaitTraj {
	std::array<LegJointAngles, 4> legs; // LF, LR, RR, RF
};

// Enums (for easily accessing structs)
enum LegID {
	LF, // 0
	LR, // 1
	RR, // 2
	RF  // 3
};
// This arrays lets us index through the legs 
constexpr LegID LEG_ORDER[4] = { LF, LR, RR, RF };

enum GaitID {
	WALK,  // 0
	TROT,  // 1
	GALLOP,// 2
	CANTER // 3
	
};
constexpr GaitID GAIT_ORDER[4] = { WALK, TROT, GALLOP,CANTER };

// Function prototypes (assuming everything is public for now, can change later)
void printGaitPath(GaitPath path);
void printThetas(ThetaTraj theta);
GaitPath createGaitPath(double lStance, double hMax, Vector xStance, Vector yStance, Vector xSwing, Vector ySwing);
GaitPath adjustPathOrigin(GaitPath path, double lStance, double hMax, double joint_height);
ThetaTraj computeIK(GaitPath path, double ground_height);
ThetaTraj shiftTrajectory(ThetaTraj thetas, int shift);

void generate_trajectory(int W); // THIS IS THE MAIN FUNCTION TO CALL FROM THE MAIN SCRIPT

// Variables we want to access in other files
extern GaitTraj gaitTrajs[4]; // To access a specific joint angle: gaitTrajs[WALK].LEGS[LF].q1
extern double V[4];
extern double gaitTime[4];

#endif

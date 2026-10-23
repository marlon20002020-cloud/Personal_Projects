// TODO: 
// Integrate with Unitree

#define _USE_MATH_DEFINES
#include <iostream>
// #include <iomanip>
#include <cmath>
// #include <limits>
// #include <numbers>
#include <vector>
#include <array>
#include <cmath> 
#include <algorithm> 
// #include <memory>
#include "unitree_legged_sdk/gait.h"

using namespace std;
using std::sin; //brings this into the namespace so we can just say "sin" instead of "std::sin"
using std::cos;
using std::atan2;

// this creates an array of GaitTraj structs (for walk, trot, gallop , canter)
GaitTraj gaitTrajs[4];

// Define global arrays for forward velocity / gait period
double V[4] = {FORWARD_VEL, 1.0, 1.5, 0.9};
double gaitTime[4] = {0.33, 0.275, 0.30, 0.30};

//*********************************************************Walk Trot Gallop Gait Functions***************************************************************************

void printGaitPath(GaitPath path) {
	// Print out the xy matrix vectors for plotting comparison in Matlab
	std::cout << "GaitPath X: " << std::endl;
	for (int i = 0; i < NUM_INTERVALS; i++) {
		std::cout << path[0][i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "GaitPath Y:" << std::endl;
	for (int i = 0; i < NUM_INTERVALS; i++) {
		std::cout << path[1][i] << ", ";
	}
	std::cout << std::endl;
}

void printThetas(ThetaTraj theta) {
	std::cout << "Theta2: " << std::endl;
	for (int i = 0; i < NUM_INTERVALS; i++) {
		std::cout << theta[1][i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "Theta3:" << std::endl;
	for (int i = 0; i < NUM_INTERVALS; i++) {
		std::cout << theta[2][i] << ", ";
	}
	std::cout << std::endl;
}

//get xy cordinates of gait path
//This return type works since we know the size of the output array
GaitPath createGaitPath(double lStance, double hMax, Vector xStance, Vector yStance, Vector xSwing, Vector ySwing) {

	int stanceLength = xStance.size();
	int swingLength = xSwing.size();
	//Create xStance and yStance from x = lStance -> x = 0

	for (int i = 0; i < (stanceLength); i++) {
		double alpha = static_cast<double>(i) / (stanceLength - 1);
		xStance[i] = (1 - alpha) * lStance;
		yStance[i] = 0;
	}

	/* Create a parameterized spline using an X spline and a Y spline */
	vector<double> xx = { 0,-0.05 * lStance, -0.08 * lStance, -0.05 * lStance, lStance / 2, 1.05 * lStance, 1.08 * lStance, 1.05 * lStance, lStance };
	vector<double> yy = { 0, 0.1 * hMax,      0.2 * hMax,      0.6 * hMax,     hMax,      0.6 * hMax,     0.2 * hMax,     0.1 * hMax,     0 };
	static int num_knots = xx.size();
	vector<double> t(num_knots);
	for (int i = 0; i < num_knots; i++) { // assign time-axis of spline to be between 0 and 1
		t[i] = static_cast<double>(i) / (num_knots - 1);
	}
	//Create an x spline and a y spline object
	cubic_spline spline_x(t, xx);
	cubic_spline spline_y(t, yy);
	//Fill in our swing trajectory along this spline path        
	for (int i = 0; i < swingLength; i++) {
		double d = static_cast<double>(i) / (swingLength);
		xSwing[i] = spline_x(d);
		ySwing[i] = spline_y(d);
	}
	//Create xy matrix for the foot trajectory  - combine stance and swing phase
	GaitPath xy;
	for (int ii = 0; ii < NUM_INTERVALS; ii++) {
		if (ii < swingLength) {
			xy[0][ii] = xSwing[ii];
			xy[1][ii] = ySwing[ii];
		}
		else {
			int idx = ii - swingLength;
			xy[0][ii] = xStance[idx];
			xy[1][ii] = yStance[idx];
		}
	}

	return xy;

}

GaitPath adjustPathOrigin(GaitPath path, double lStance, double hMax, double joint_height) {
	// We want to place the origin at the thigh joint, so the leg swings from 0 to -lStance to lStance and back to 0.
	 //Also, then the y position becomes a negative value (set to be the default stand height of the robot)

	 //Shift the y-axis down so that the ground begins at -joint_height (and thus joint_height is y = 0)
	 //Shift the x-axis so that the joint is the origin point
	for (int i = 0; i < NUM_INTERVALS; i++) {
		path[0][i] = path[0][i] - lStance / 2.0;
		path[1][i] = path[1][i] - joint_height;
	}

	return path;
}

ThetaTraj computeIK(GaitPath path, double ground_height) {
	//Right now this just does a planar manipulator computation for Th2 and Th3 and sets Th1 to 0.0 always.

	ThetaTraj theta;
	double x_target = 0;
	double y_target = -ground_height;
	double unitree_stand_theta2 = 0.64;
	int best_norm_idx = 0;
	double smallest_norm = 10000;

	//Loop through the entire path, one point at a time
	//While we compute the new thetas, look for the x/y coordinate closest to the default stand position so we can adjust after
	for (int i = 0; i < NUM_INTERVALS; i++) {
		double x = path[0][i];
		double y = path[1][i];
		//Find Theta3
		double d = (x * x + y * y - LINK2_LENGTH * LINK2_LENGTH - LINK3_LENGTH * LINK3_LENGTH) / (2 * LINK2_LENGTH * LINK3_LENGTH);
		if (d > 1.0) {
			throw std::invalid_argument("lStance is too large; leg cannot reach the desired point in the path.");
		}
		theta[2][i] = atan2(std::sqrt(1 - (d * d)), d);
		//Find Theta2
		double theta_ref = atan2(y, x);
		double alpha = atan2(LINK3_LENGTH * sin(theta[2][i]), LINK2_LENGTH + LINK3_LENGTH * cos(theta[2][i]));
		theta[1][i] = theta_ref - alpha;
		//Set Theta1 to 0
		theta[0][i] = 0.0;
		//Save the distance this point is from the default stand position (L2 norm)
		double norm = std::sqrt((x - x_target) * (x - x_target) + (y - y_target) * (y - y_target));
		if (norm < smallest_norm) {
			smallest_norm = norm;
			best_norm_idx = i;
		}
	}
	//std::cout << "Computed Stand Angle: " << theta[1][best_norm_idx] << " At index: " << best_norm_idx << std::endl;
	//std::cout << "Computed Stand Position x: " << path[0][best_norm_idx] << " y: " << path[1][best_norm_idx] << std::endl;
	//Now that we know where the leg is closest to "standing" based on the norm, we can adjust our computed thetas to the Unitree frame.
	double theta2_computed_stand = theta[1][best_norm_idx];
	for (int i = 0; i < NUM_INTERVALS; i++) {
		theta[1][i] = -theta[1][i] + theta2_computed_stand + unitree_stand_theta2;
		theta[2][i] = -theta[2][i]; // This is a shift by pi essentially.
		if ((theta[1][i] < THIGH_JOINT_MIN) || (theta[1][i] > THIGH_JOINT_MAX)) {
			throw std::invalid_argument("Thigh joint angle is out of range!");
		}
		else if ((theta[2][i] < CALF_JOINT_MIN) || (theta[2][i] > CALF_JOINT_MAX)) {
			throw std::invalid_argument("Calf joint angle is out of range!");
		}
	}

	return theta;
}

ThetaTraj shiftTrajectory(ThetaTraj thetas, int shift) {

	// Rotate all joints in the ThetaTraj object
	for (int i = 0; i < 3; i++) {
		std::array<double, NUM_INTERVALS>& row = thetas[i]; // ampersand references the row by row
		std::rotate(row.begin(), row.begin() + shift, row.end());
	}
	return thetas;
}
//********************************************************End of Walk Trot Gallop Gait Functions****************************************************************************

void generate_trajectory(int gait_num) { // gait_num is an int from 0-3 for walk, Trot, Gallop, Canter
	// Define constants that define the gaits
// int main() {
	// const int gait_num = 2;// 0,1,2,3 -> Runs code for Walk, Trot, Gallop, Canter
	
	// LF LR RR RF
	double phi[4][4] = { { 0, 0.25, 0.5, 0.75 },{0, 0.5, 0, 0.5},{0.0, 0.2, 0.4, 0.6},{0.1,0.25,0.6,0.35} }; // Row Order: Walk, Trot, Gallop, Canter
	double beta[3] = { 0.75, 0.5, 0.40 };
	double betaC[4] = { 0.5,0.35,0.4,0.75 }; // Canter Gait 

	double swingFract[3];
	double swingFractC[4];
	for (int i = 0; i < 3; i++) {
		swingFract[i] = 1 - beta[i];
		swingFractC[i] = 1 - betaC[i];
	}
	swingFractC[3] = 1 - betaC[3]; //Lazy and did not want to write another for loop

	// Path settings
	double hMax = 0.10;
	double th2Stand = 0.67 - M_PI / 2.0;
	double th3Stand = -1.3;
	double default_y_distance = LINK2_LENGTH * sin(th2Stand) + LINK3_LENGTH * sin(th2Stand + th3Stand); // this is y height in the stand position
	double joint_height_from_ground = 0 - default_y_distance;
	// std::cout << "joint_height_from_ground: " << joint_height_from_ground << std::endl;

if (gait_num<3){
	double lStance = 0.0;
	int swingLength = static_cast<int>(std::round(swingFract[gait_num] * NUM_INTERVALS));
	int stanceLength = static_cast<int>(std::round((1 - swingFract[gait_num]) * NUM_INTERVALS));
	Vector xSwing(swingLength);
	Vector ySwing(swingLength);
	Vector xStance(stanceLength);
	Vector yStance(stanceLength);

	lStance = V[gait_num] * gaitTime[gait_num] * beta[gait_num];
	GaitPath xy = createGaitPath(lStance, hMax, xStance, yStance, xSwing, ySwing);
	xy = adjustPathOrigin(xy, lStance, hMax, joint_height_from_ground);
	// printGaitPath(xy);
	ThetaTraj thetas = computeIK(xy, joint_height_from_ground);
	// //printThetas(thetas);


	// 	// Generate trajectories for each leg by circular shifting the arrays, then save that data to structs
	for (int j = 0; j < 4; j++) {
		//Shift arrays
		int shift = std::round(NUM_INTERVALS * phi[gait_num][j]);
		ThetaTraj thetas_shifted = shiftTrajectory(thetas, shift);
		//Save data
		LegJointAngles leg;
		leg.q1 = thetas_shifted[0];
		leg.q2 = thetas_shifted[1];
		leg.q3 = thetas_shifted[2];
		gaitTrajs[gait_num].legs[j] = leg; // can reference this gaitTraj later with enums

	};
	//std::cout << "Completed trajectory generation!" << std::endl;
	// for (int i = 0; i < NUM_INTERVALS; i++) {
	// 	std::cout << gaitTrajs[gait_num].legs[LF].q3[i] << ", ";
	// }
}
else{
	// ***********************************************************Canter Gait*******************************************************************************
	double lStance = 0;
	for (int i = 0; i < 4; i++) {
		lStance = V[3] * gaitTime[3] * betaC[i];
		int swingLength = static_cast<int>(std::round(swingFractC[i] * NUM_INTERVALS));
		int stanceLength = static_cast<int>(std::round((1 - swingFractC[i]) * NUM_INTERVALS));
		Vector xSwing(swingLength);
		Vector ySwing(swingLength);
		Vector xStance(stanceLength);
		Vector yStance(stanceLength);
		// Generate a baseline desired path for the leg to follow (assumes phi = 0);
		GaitPath xy = createGaitPath(lStance, hMax, xStance, yStance, xSwing, ySwing);

		// Adjust the path so that (0,0) is centered at the thigh joint - so it swings from -lStance to lStance at a height of -joint_height
		xy = adjustPathOrigin(xy, lStance, hMax, joint_height_from_ground);
		//printGaitPathC(xyC);

		// Run IK on the path to get the joint angles (and convert to Unitree space)
		ThetaTraj thetas = computeIK(xy, joint_height_from_ground);
		// printThetas(thetas);

		// Generate trajectories for each leg by circular shifting the arrays, then save that data to structs
		//for (int j = 0; j < 4; j++) {
			// Shift arrays
			int shift = std::round(NUM_INTERVALS * phi[3][i]);
			ThetaTraj thetas_shifted = shiftTrajectory(thetas, shift);
			// Save data
			LegJointAngles leg;
			leg.q1 = thetas_shifted[0];
			leg.q2 = thetas_shifted[1];
			leg.q3 = thetas_shifted[2];
			gaitTrajs[gait_num].legs[i] = leg; // can refernce this gaitTraj later with enums

		//}
		//Test print line for troubleshooting
		
	}
std::cout << gaitTrajs[gait_num].legs[LR].q1[1] << ", ";
}

	// Print tests



	// std::cout << "Theta1:" << std::endl;
	// for (int i = 0; i < NUM_INTERVALS; i++) {
	// 	std::cout << gaitTrajs[CANTER].legs[LR].q1[i] << ", ";
	// }
	// std::cout << std::endl;
	// std::cout << "Theta2:" << std::endl;
	// for (int i = 0; i < NUM_INTERVALS; i++) {
	// 	std::cout << gaitTrajs[CANTER].legs[LR].q2[i] << ", ";
	// }
	// std::cout << std::endl;
	// std::cout << "Theta3:" << std::endl;
	// for (int i = 0; i < NUM_INTERVALS; i++) {
	// 	std::cout << gaitTrajs[CANTER].legs[LR].q3[i] << ", ";
	// }
	// return 0;
}

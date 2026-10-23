/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

// motiontime measured in milliseconds; RobotControl runs at dt (s) = 0.002 -> 2 ms per tick

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h> // Added for usleep()
#include <cmath>
#include "unitree_legged_sdk/gait.h"

constexpr int STAND_TIME_MS = 10000;
constexpr int GAIT_DURATION_MS = STAND_TIME_MS + 60000; // stand time will run first so this really just makes a timestamp
constexpr int FILTER_SIZE = 20; // circular buffer size for averaging yaw turn rate
constexpr int YAW_GAIN = 0.00; // set to 0 to not use control (1.00 for walk control)

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void setParam(int p); // This function let's us pass an argument to the class
  void UDPRecv();
  void UDPSend();
  void RobotControl();
  void stand();
  void MoveToStandNew();
  float UpdateBuffer();
  void moveFromStandToGait(int current_time, int total_duration);
  void moveFromGaitToStand(int current_time, int end_time, int total_duration);

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[3] = {0};
  float qDes[3] = {0};
  float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01 - 500 Hz frequency
  int gait_counter = 0; // this will count the iterations of the loop
  int print_counter = 0; // used to print less frequently so that we do not get latency from printing
  double robot_start_pos[12] = {0};
  double gait_start_pos[12] = {0};
  double gait_final_pos[12] = {0};
  double stand_pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
  int gait_num = 0;
  float yaw_standard = 0.0; // this sets the yaw "offset" based on the stand start position

  // This marks how many control loops to wait before sending a new trajectory. Without it we send trajectories at the control loop rate (1 kHz) 
  // instead of the rate that we assumed in gaitV3.cpp (which is NUM_INTERVALS / gaitTime (gives number of trajectories per second))
  // NOTE: floor gives us an approximation which errs on the slow side
  int timingDelay = 1; // gets set in setParams
  float circularBuffer[FILTER_SIZE] = {0};
  int filterHead = 0;
  bool isFirstUpdate = true;
  
};

void Custom::setParam(int p)
{
    this->gait_num = p;

    // Protect against invalid gaitTime values
    double gt = gaitTime[this->gait_num];
    if (gt <= 0.0) {
        gt = 1.0;
    }

    // number of trajectory updates per second assumed by gaitV3 = NUM_INTERVALS / gt
    double delay_d = 1000.0 / (NUM_INTERVALS / gt);
    int delay_i = static_cast<int>(std::floor(delay_d));
    delay_i = delay_i * 2; // double the number since each loop iteration incease motiontime by 2
    // Always ensure at least 1 loop delay (and avoid zero)
    this->timingDelay = std::max(1, delay_i);
}

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

float Custom::UpdateBuffer() {
  // Updates the circular buffer and returns the averaged value
  float current_yaw = state.imu.rpy[2];
  
  // if the buffer is empty, fill with the first value, otherwise update the buffer
  if (isFirstUpdate == true) {
    for (int i = 0; i < FILTER_SIZE; i++) {
      circularBuffer[i] = current_yaw;
    }
    isFirstUpdate = false;
  } else {
    circularBuffer[filterHead] = current_yaw;
  }
  // increment the filterHead for next time
  filterHead++;
  if (filterHead >= FILTER_SIZE) {
    filterHead = 0; //reset to beginning of array
  }
  // now calculate and return the average yaw
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum = sum + circularBuffer[i];
  }
  return sum / FILTER_SIZE;
}
  
void Custom::MoveToStandNew() {
  double t = motiontime * dt;

  auto PD = [&](int id, double qref, double kp, double kd){
      cmd.motorCmd[id].q  = qref;
      cmd.motorCmd[id].dq = 0.0;
      cmd.motorCmd[id].Kp = kp;
      cmd.motorCmd[id].Kd = kd;
      cmd.motorCmd[id].tau = 0.0f;
  };

  // PHASE 0 — OPEN HIPS
  static bool initHip = false;
  static double FR0_init, FL0_init, RR0_init, RL0_init;

  double FR0_target = 0.0, FL0_target = 0.0;
  double RR0_target = 0.0, RL0_target = 0.0;

  if(!initHip){
      FR0_init = state.motorState[FR_0].q;
      FL0_init = state.motorState[FL_0].q;
      RR0_init = state.motorState[RR_0].q;
      RL0_init = state.motorState[RL_0].q;
      initHip = true;
      printf("PHASE 0: Opening hips...\n");
  }

  double Topen = 1.5;
  double r0 = t / Topen;  if(r0 > 1.0) r0 = 1.0;

  PD(FR_0, (1-r0)*FR0_init + r0*FR0_target, 40, 3);
  PD(FL_0, (1-r0)*FL0_init + r0*FL0_target, 40, 3);
  PD(RR_0, (1-r0)*RR0_init + r0*RR0_target, 40, 3);
  PD(RL_0, (1-r0)*RL0_init + r0*RL0_target, 40, 3);

  if(r0 < 1.0){ 
    udp.SetSend(cmd);
    return;
  }

  // PHASE 1 — NEUTRAL POSE
  static bool initNeutral = false;
  static double FR1_init, FL1_init, RR1_init, RL1_init;
  static double FR2_init, FL2_init, RR2_init, RL2_init;

  double hipNeutral  =  0.30;
  double kneeNeutral = -0.70;

  if(!initNeutral){
      FR1_init = state.motorState[FR_1].q;
      FL1_init = state.motorState[FL_1].q;
      RR1_init = state.motorState[RR_1].q;
      RL1_init = state.motorState[RL_1].q;

      FR2_init = state.motorState[FR_2].q;
      FL2_init = state.motorState[FL_2].q;
      RR2_init = state.motorState[RR_2].q;
      RL2_init = state.motorState[RL_2].q;

      initNeutral = true;
      printf("PHASE 1: Moving to neutral...\n");
  }

  double Tneutral = 2.0;
  double r1 = (t - Topen) / Tneutral;
  if(r1 < 0) r1 = 0; if(r1 > 1) r1 = 1;

  PD(FR_1, (1-r1)*FR1_init + r1*hipNeutral,  50, 3);
  PD(FL_1, (1-r1)*FL1_init + r1*hipNeutral,  50, 3);
  PD(RR_1, (1-r1)*RR1_init + r1*hipNeutral,  50, 3);
  PD(RL_1, (1-r1)*RL1_init + r1*hipNeutral,  50, 3);

  PD(FR_2, (1-r1)*FR2_init + r1*kneeNeutral, 50, 4.5);
  PD(FL_2, (1-r1)*FL2_init + r1*kneeNeutral, 50, 4.5);
  PD(RR_2, (1-r1)*RR2_init + r1*kneeNeutral, 50, 4.5);
  PD(RL_2, (1-r1)*RL2_init + r1*kneeNeutral, 50, 4.5);

  if(r1 < 1.0) { 
    udp.SetSend(cmd);
    return;
  }

  // PHASE 2 — MOVE KNEES UP
  static bool initKnee = false;
  static double FR2_q0, FL2_q0, RR2_q0, RL2_q0;
  static double FR1_q0, FL1_q0, RR1_q0, RL1_q0;


  if(!initKnee){
      FR2_q0 = state.motorState[FR_2].q;
      FL2_q0 = state.motorState[FL_2].q;
      RR2_q0 = state.motorState[RR_2].q;
      RL2_q0 = state.motorState[RL_2].q;
      FR1_q0 = state.motorState[FR_1].q;
      FL1_q0 = state.motorState[FL_1].q;
      RR1_q0 = state.motorState[RR_1].q;
      RL1_q0 = state.motorState[RL_1].q;
      initKnee = true;
  }

  double K_front = 75.0;   // stiffness
  double K_rear = 90.0;   // stiffness
  double D = 4.0;    // damping
  double hip_final = 0.67;
  double knee_final = -1.3;
  
  double Tfinal = 2.0;
  double r2 = (t - (Topen + Tneutral)) / Tfinal;
  if(r2 < 0) r2 = 0; if(r2 > 1) r2 = 1;
  
  PD(FR_1, (1-r2)*FR1_q0+r2*hip_final, K_front, D);
  PD(FL_1, (1-r2)*FL1_q0+r2*hip_final, K_front, D);
  PD(RR_1, (1-r2)*RR1_q0+r2*hip_final, K_rear, D);
  PD(RL_1, (1-r2)*RL1_q0+r2*hip_final, K_rear, D);
  PD(FR_2, (1-r2)*FR2_q0+r2*knee_final, K_front, D);
  PD(FL_2, (1-r2)*FL2_q0+r2*knee_final, K_front, D);
  PD(RR_2, (1-r2)*RR2_q0+r2*knee_final, K_rear, D);
  PD(RL_2, (1-r2)*RL2_q0+r2*knee_final, K_rear, D);
  
  if (r2 < 1.0) {
    udp.SetSend(cmd); // no return so we always send it after complete
  }
  if (r2 >= 1.0) {
    PD(FR_1, hip_final, K_front, D);
    PD(FL_1, hip_final, K_front, D);
    PD(RR_1, hip_final, K_rear, D);
    PD(RL_1, hip_final, K_rear, D);
    PD(FR_2, knee_final, K_front, D);
    PD(FL_2, knee_final, K_front, D);
    PD(RR_2, knee_final, K_rear, D);
    PD(RL_2, knee_final, K_rear, D);
  }
}

void Custom::RobotControl()
{
  
  udp.GetRecv(state);
  if (print_counter % 50 == 0) { // 50 * 2 ms = 100 ms
    // printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);
  }
  print_counter++;

 

  /* CUSTOM CODE */

  // Kp and Kd
  double K_hip = 40.0; // 40
  double K_front = 70.0;    // stiffness //70 
  double K_rear = 90.0;     // stiffness //90
  double D = 4.0;           // damping

  auto PD = [&](int id, double qref, double kp, double kd){
    cmd.motorCmd[id].q  = qref;
    cmd.motorCmd[id].dq = 0.0;
    cmd.motorCmd[id].Kp = kp;
    cmd.motorCmd[id].Kd = kd;
    cmd.motorCmd[id].tau = 0.0f;
  };

  // fill the initial position array
  if (motiontime == 0) {
    for (int j = 0; j < 12; ++j) {
      robot_start_pos[j] = state.motorState[j].q;
    } 
  }

  // Stand up the robot and set the yaw standard reference
  if (motiontime > 0 && motiontime < STAND_TIME_MS) {
    MoveToStandNew();
    yaw_standard = state.imu.rpy[2];
  }

  // Set the robot to execute the gait for a set amount of time
  if (motiontime > (STAND_TIME_MS) && motiontime < GAIT_DURATION_MS) {
    // Only send a command periodically so that we send the right number of angles in our trajectory based on the control loop timing
    if (motiontime % timingDelay == 0) {
      int cycle_iter = gait_counter % NUM_INTERVALS;
      // if (motiontime % 50 == 0) { // print data less often
	      	    // printf("roll: %f, pitch: %f, yaw: %f, yaw_adj: %f\n", state.imu.rpy[0],  state.imu.rpy[1], state.imu.rpy[2], yaw_standard - state.imu.rpy[2]);
     // }
      // Determine the correction factor for the hip joint if yaw is drifting 
      // (meaning the robot is turning left or right as it moves)
      // This forms a control loop as the main control loop processes
      float average_yaw = UpdateBuffer();
      float left_front_hip_angle = 0.0;
      float right_front_hip_angle = 0.0;
      bool changeLeftHip = false;
      bool changeRightHip = false;
      float MAX_JOINT_ANGLE = 10 * (6.28) / 360; // degrees to rads
      
      // if the robot is listing to the left, we want to move the left hip slightly
      // UNCOMMENT THE BELOW IFS FOR WALK YAW CONTROL
      // if (average_yaw > 0.03) {
       // left_front_hip_angle = -YAW_GAIN * average_yaw;
       // changeLeftHip = true;
      //}
      //else if (average_yaw < -0.03) {
       // right_front_hip_angle = -YAW_GAIN * average_yaw;
       // changeRightHip = true;
      //}
      //if (average_yaw > 0.03 || average_yaw < -0.03) {
      //  left_front_hip_angle = -YAW_GAIN * average_yaw;
       // right_front_hip_angle = -YAW_GAIN * average_yaw;
       // changeLeftHip = true;
       // changeRightHip = true;
      //}
      // clamp values to -5 degrees (0.087 rads)
      if (left_front_hip_angle > MAX_JOINT_ANGLE)
        left_front_hip_angle =  MAX_JOINT_ANGLE;
      if (left_front_hip_angle < -MAX_JOINT_ANGLE)
        left_front_hip_angle = -MAX_JOINT_ANGLE;
      if (right_front_hip_angle < -MAX_JOINT_ANGLE) {
        right_front_hip_angle = -MAX_JOINT_ANGLE;
      if (right_front_hip_angle > MAX_JOINT_ANGLE)
        left_front_hip_angle = MAX_JOINT_ANGLE;
      }
      if (motiontime % 10 == 0) {
       printf("average yaw: %f, left hip: %f, right hip: %f\n", average_yaw, left_front_hip_angle, right_front_hip_angle);
      }
      // update hip joints
      if (changeRightHip == true) {
        PD(FR_0, right_front_hip_angle, K_hip, D);
      } else {
        PD(FR_0, gaitTrajs[gait_num].legs[RF].q1[cycle_iter], K_hip, D);
      }
      
      if (changeLeftHip == true) {
        PD(FL_0, left_front_hip_angle, K_hip, D);
      } else {
        PD(FL_0, gaitTrajs[gait_num].legs[LF].q1[cycle_iter], K_hip, D);
      }
      
      // PD(FR_0, -0.1, K_hip, D);
      // PD(FL_0, 0.1, K_hip, D);
      PD(RR_0, gaitTrajs[gait_num].legs[RR].q1[cycle_iter], K_hip, D);
      PD(RL_0, gaitTrajs[gait_num].legs[LR].q1[cycle_iter], K_hip, D);
      PD(FR_1, gaitTrajs[gait_num].legs[RF].q2[cycle_iter], K_front, D);
      PD(FR_2, gaitTrajs[gait_num].legs[RF].q3[cycle_iter], K_front, D);     
      PD(FL_1, gaitTrajs[gait_num].legs[LF].q2[cycle_iter], K_front, D);
      PD(FL_2, gaitTrajs[gait_num].legs[LF].q3[cycle_iter], K_front, D);
      PD(RR_1, gaitTrajs[gait_num].legs[RR].q2[cycle_iter], K_rear, D);
      PD(RR_2, gaitTrajs[gait_num].legs[RR].q3[cycle_iter], K_rear, D);
      PD(RL_1, gaitTrajs[gait_num].legs[LR].q2[cycle_iter], K_rear, D);
      PD(RL_2, gaitTrajs[gait_num].legs[LR].q3[cycle_iter], K_rear, D);
      gait_counter++;
    }
  }

  // Once gait is timed out, record the last position of all the legs so we can soft stop
  if (motiontime == GAIT_DURATION_MS) {
    for (int i = 0; i < 12; i++) {
      gait_final_pos[i] = state.motorState[i].q;
    }
  }

  // When complete, soft stop to a stand pos
  if (motiontime > GAIT_DURATION_MS && motiontime < (GAIT_DURATION_MS+500)) {
    // second arg is last num of if statement, the last arg is duration
    moveFromGaitToStand(motiontime, GAIT_DURATION_MS+500, 500); 
  }
  // Then hold a stand pos
  if (motiontime > GAIT_DURATION_MS+500) {
    stand();
  }

  // Safety stuff (left as is)
  if (motiontime > STAND_TIME_MS)
  {
    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 8);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
  }
  
  motiontime = motiontime + 2; // Now we are in millisecond time

  udp.SetSend(cmd);
}

void Custom::stand() {
  // Holds the stand position 
  for (int i = 0; i < 12; i++) {
      cmd.motorCmd[i].q = stand_pos[i];
  }
}

void Custom::moveFromStandToGait(int current_time, int total_duration) {
  // This is a soft start to the gait cycle, so the legs do not jump right to the start point
  // We want to move towards the first position of the gait (index 0)
  // This was not used in the final work, but could be modified with PD control and implemented.
  double percent = static_cast<double>(current_time) / total_duration;
  if (percent > 1.0) percent = 1.0;
  if (percent < 0.0) percent = 0.0;
  // shorthand reference
  auto& traj = gaitTrajs[gait_num].legs;
  // Update the legs gradually
  cmd.motorCmd[FR_0].q = (1-percent)*gait_start_pos[FR_0] + percent*traj[RF].q1[0];
  cmd.motorCmd[FR_1].q = (1-percent)*gait_start_pos[FR_1] + percent*traj[RF].q2[0];
  cmd.motorCmd[FR_2].q = (1-percent)*gait_start_pos[FR_2] + percent*traj[RF].q3[0];

  cmd.motorCmd[FL_0].q = (1-percent)*gait_start_pos[FL_0] + percent*traj[LF].q1[0];
  cmd.motorCmd[FL_1].q = (1-percent)*gait_start_pos[FL_1] + percent*traj[LF].q2[0];
  cmd.motorCmd[FL_2].q = (1-percent)*gait_start_pos[FL_2] + percent*traj[LF].q3[0];

  cmd.motorCmd[RR_0].q = (1-percent)*gait_start_pos[RR_0] + percent*traj[RR].q1[0];
  cmd.motorCmd[RR_1].q = (1-percent)*gait_start_pos[RR_1] + percent*traj[RR].q2[0];
  cmd.motorCmd[RR_2].q = (1-percent)*gait_start_pos[RR_2] + percent*traj[RR].q3[0];

  cmd.motorCmd[RL_0].q = (1-percent)*gait_start_pos[RL_0] + percent*traj[LR].q1[0];
  cmd.motorCmd[RL_1].q = (1-percent)*gait_start_pos[RL_1] + percent*traj[LR].q2[0];
  cmd.motorCmd[RL_2].q = (1-percent)*gait_start_pos[RL_2] + percent*traj[LR].q3[0];
}

void Custom::moveFromGaitToStand(int current_time, int end_time, int total_duration) {
  // This is a soft start to the gait cycle, so the legs do not jump right to the start point
  double percent = double(end_time - current_time) / total_duration;
  percent = std::min(std::max(percent, 0.0), 1.0); // hold between 0 and 1

  for (int i = 0; i < 12; i++) {
    cmd.motorCmd[i].q = (1.0 - percent)*gait_final_pos[i] + percent*stand_pos[i];
  }
}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  
  // Tell which gait to run
  int gait_num = 1; // 0 = walk, 1 = trot, 2 = gallop, 3 = canter
  custom.setParam(gait_num);
  
  // Generate the gait trajectory outside of the loop for testing - trajectories saved to gaitTrajs global struct
  generate_trajectory(gait_num);

  // InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}

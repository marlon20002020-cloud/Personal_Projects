## Source Code
These are the executables and library files that will be used to accomplish gaits. They are as follows:
- robot_run_walk.cpp: runs the walk gait
- robot_run_trot.cpp: runs the trot gait
- robot_run_gallop.cpp: runs the gallop gait
- robot_run_canter.cpp: runs the canter gait
All files should be executed from the Go1 resting in the home position (laying flat on floor), and it will accomplish a standup routine prior to executing the gait.
Walk includes a yaw-correction control loop to prevent the robot from listing to the left, as is very common.

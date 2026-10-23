# What to do with these files
Add .h files to unitree_ros/unitree_controller/include  
Add .cpp files to unitree_ros/unitree_controller/src   
Run the .m file to generate the gait trajectories in C++-copyable format that can be pasted into the body.cpp file  
Update CMakeLists for these src files and run catkin_make, similar to the instructions for the real robot  


# to run a simulation  
In terminal one: 
'source /opt/ros/noetic/setup.bash'  
'source ~/catkin_ws/devel/setup.bash'  
start ros up  
'roscore'  

in terminal two:  
'source /opt/ros/noetic/setup.bash'  
'source ~/catkin_ws/devel/setup.bash'  
to run Rviz (optional)  
'cd ~/catkin_ws'  
'roslaunch go1_description go1_rviz.launch'  
to run Gazebo  
'roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth' can also do stairs or space instead of earth  

in terminal three:  
'source /opt/ros/noetic/setup.bash'  
'source ~/catkin_ws/devel/setup.bash'  
now to start the controller sequence with leg stand, etc.  
'cd ~/catkin_ws'  
`rosrun unitree_controller unitree_servo` # to stand up the robot  
`rosrun unitree_controller unitree_move_kinetic` # robot moves in a circle  
`rosrun unitree_controller walk_gait` # if you update CMakeLists.txt to run the walk file for example  

# Misc.  
You might need to rebuild the packages if it can't find a package or if you get RLException  
`cd ~/catkin_ws`  
`rm -rf build devel`  
`catkin_make`  
`source ~/catkin_ws/devel/setup.bash`  
`source /opt/ros/noetic/setup.bash`  


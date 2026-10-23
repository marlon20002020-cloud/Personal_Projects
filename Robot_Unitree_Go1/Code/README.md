# To Setup a Unitree Go1 in Lab
1. Plug in a charged battery
2. Turn it on, and wait a minute for it to stand up on its own. This is high state mode.
3. To enter low state, grab the controller and select L2+A, L2+A, L2+B, L2+L1+START.
4. The robot should be laying on the floor now, and emitting a local wifi connection you can link to on your Linux machine.

# How to Run Gaits On a Physical Robot
1. Download the Unitree Go1 repos, including unitree_ros, unitree_ros_to_real, and unitree_legged_sdk to your catkin_ws/src directory.
2. Add the files from this repo's `src` folder to the unitree_legged_sdk/example folder.
3. Add the files from this repo's `include` folder to the unitree_legged_sdk/include/unitree_legged_sdk folder.
4. Update the CMakeList located in ../catkin_ws/src/unitree_legged_sdk to add on these new executables and source code.
5. Add the executable paths along with the gaitV3.cpp library to the CMakeLists.txt file so that these compile.
5a. To add a executable, for example:

```
add_executable(example_position_gaits example/robot_run_walk.cpp example/gaitV3.cpp)
target_link_libraries(robot_run_walk ${EXTRA_LIBS})
```

6. `cd ~/catkin_ws`
7. `catkin_make` -> if there are syntax errors or compiler errors, they likely stem from the CMakeLists.txt file.
8. `cd ..catkin_ws/src/unitree_legged_sdk/example`
9. Compile the code:
    
```
g++ robot_run_walk.cpp \
     gaitV3.cpp \
    -I ../include \
    -L ../lib/cpp/amd64 \
    -lunitree_legged_sdk \
    -lpthread \
    -o robot_run_walk
```
10. Pair to the Unitree Go1, now in low state mode, through wifi.
11. Execute the file: `./robot_run_walk`


    

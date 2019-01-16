### Robot Interface

This is a software package used for controlling and learning skills on the Franka Emika Panda Research Robot Arm.

## Requirements

* A Control PC with the Ubuntu 16.04 Realtime Kernel capable of communicating with the Franka Robot at 1 kHz over ethernet.
* ROS Kinetic 

## Installation

1. Clone this repository and its submodules using the following command: `git clone --recurse-submodules https://github.com/iamlab-cmu/robot-interface.git`. This may require you to upload your ssh key to github, which you can learn about [here](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/).
2. Change directory into the repository: `cd robot-interface`
3. Then change directory into the libfranka folder: `cd libfranka`
4. Open up the CMakeLists.txt file in the libfranka folder and edit it so that on line 129, option BUILD_TESTS is turned off instead of on `option(BUILD_TESTS "Build tests" OFF)`.
5. Make a build directory and enter it: `mkdir build && cd build`
6. Run the following command: `cmake -DCMAKE_BUILD_TYPE=Release .. && make`
7. After libfranka has completed building, return to the robot-interface folder: `cd ../..`
8. Copy the cmake folder in the libfranka folder to the robot-interface folder: `cp -R libfranka/cmake .`
9. Make a build directory and enter it: `mkdir build && cd build`
10. Run the following command: `cmake -DCMAKE_BUILD_TYPE=Release .. && make`
11. Once it has finished building, you should see an application named `test_iam_robolib` in the build folder.
12. Next, we will have to build the ros node. Change directory to the ros folder: `cd ../src/catkin_ws`
13. Make sure that you have installed ROS Kinetic already and have added the `source /opt/ros/kinetic/setup.bash` into your `~/.bashrc` file.
14. Run catkin_make using the command `catkin_make`.
15. Once catkin_make has finished there should be a build and devel folder in the catkin_ws folder.

## Running

1. Make sure that both the user stop and the brakes of the Franka robot have been opened.
2. Open up 3 separate terminals.
3. In one terminal, navigate to the build folder in the robot-interface folder `cd /path/to/robot-interface/build/`. In the other 2 terminals, navigate to the catkin_ws folder `cd /path/to/robot-interface/src/catkin_ws`.
4. In the two terminals inside the catkin_ws folder, source the devel/setup.bash file. `source devel/setup.bash`.
5. In the first terminal, start the main communication program with the franka with `./test_iam_robolib`.
6. In the second terminal, start the ros action server with the command: `roslaunch franka_action_lib execute_skill_action_server_node.launch`.
7. Finally, in the third terminal, you can run one of the many scripts located in the folder `/path/to/robot-interface/src/catkin_ws/src/franka_action_lib/scripts` using the command `rosrun franka_action_lib ` followed by the script name. Go through the individual scripts in order to see what they do.
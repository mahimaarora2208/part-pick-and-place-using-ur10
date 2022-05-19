# part-pick-and-place-using-ur10
This repository contains implementation for picking and placing parts in a workcell in an inductrial setting. It also uses turtlebot to dynamically retrieve the place  location of the part using Aruco. 

`cd ~/catkin_ws/src`
- `git clone`
- Edit your `.bashrc` and add:
  - `export GAZEBO_MODEL_PATH=/absolute/path/to/pickandplace_spring2022/workcell_809e/models:$GAZEBO_MODEL_PATH`
  - `alias kittingrqt='rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller robot_description:=/ariac/kitting/robot_description'`
- `source ~/.bashrc`
- `cd ~/catkin_ws/src/pickandplace_spring2022`
- `rosdep install --from-paths . --ignore-src --rosdistro melodic -y`
- `catkin build`
- `source ~/.bashrc`
- Check the simulation environment loads with: `roslaunch workcell_809e workcell.launch`



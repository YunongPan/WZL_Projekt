# wzl_projekt
This is a project for WZL RWTH, which is used for the development of an adaptive safety concept for UR10 robots.

## Installation
1. To test the project please clone this workspace first.  
  
`cd ~`  
  
`git clone https://github.com/YunongPan/wzl_projekt.git`  
  
2. Install dependencies  
  
`cd ~/wzl_projekt`  
  
`rosdep install --from-paths src --ignore-src -r -y`  
  
3. Build the workspace  
  
`catkin_make`  
  
## Testing  
1. Source env  
  
`source devel/setup.bash`  
  
2. Add gazebo plugin  
  
`export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/wzl_projekt/src/gazebo_plugin/plannar_mover_plugin/build`  
  
3. Copy 

Open `~/wzl_projekt/src/gazebo_model`. Copy `human_male_1` folder into `~/.gazebo/models`.  
  
4. Install teleop_twist_keyboard  
  
PLease see: http://wiki.ros.org/teleop_twist_keyboard  
  
3. Launch simulation  
  
`roslaunch demonstrator_gazebo demonstrator_world.launch`  
  
4. Use keyboard to control the person  
  
Usage please see: https://github.com/ros-teleop/teleop_twist_keyboard  
  
5. Run the distance feedback node  
  
Open another terminal, and:  
  
`source ~/wzl_projekt/devel/setup.bash`  
  
`rosrun demonstrator_preprocessing demonstrator_preprocessing.py`  




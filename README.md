# RoboticsWorkshops
Robotic's Workshop python scripts and work


 ## Useful commands: ##


### roslaunch Commmands ####

- roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/empty.world *// Launch Gazebo Simulator.* 

- roslaunch kobuki_keyop keyop.launch *// Keyboard teleop.*

- rostopic list *// List current availble topics.*

- rostopic echo /odom *// Views odomotry data, change odom for any topic data you want to view.*

- roslaunch video_stream_opencv camera.launch video_stream_provider:=1  *// Provide the camera from webcam.*

- rqt_image_view. *// Display image streams.*

- roslaunch turtlebot_rviz_launchers view_robot.launch *// Starts the camera on the real robot.*


### Python Scripting Commmands ###

*// Create a workspace.*

- mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src; catkin_init_workspace .;

*// Create a package assuming dependencies suitable.*

- catkin_create_pkg commanding_velocity rospy std_msgs geometry_msgs

*// Create a scripts directory for python ROS node.*

- mkdir ~/catkin_ws/src/commanding_velocity/scripts

*// Build the workspace.*

- cd ~/catkin_ws catkin_make

*// Start Spyder (Not through the icon).*

- source ~/catkin_ws/devel/setup.bash spyder

*//Make the script executable.*

- chmod a+x **~/catkin_ws/src/commanding_velocity/scripts/command_velocity.py** // *Whatever your path is.*
- chmod +x myscript.py

*// Run the script when cd'd in.*

- ./**myscript.py** *// Whatever your path is.*

### Linux Commmands ###

- [Ctrl-C]       *// Stop a currently running command.*

- [Tab]          *// Useful to auto-complete a command.*

- [Up] and [Down] *// Browse the history of previously typed commands.*


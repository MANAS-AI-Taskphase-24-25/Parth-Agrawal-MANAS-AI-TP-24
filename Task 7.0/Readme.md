# Task 7.0 -  beROSgar 2

## Description
Tim was returning back from the volleyball match, but seems to have lost his way, can you help him out?

### Overview
Timothy has the pictures of the teeth samples collected though, the only hurdle is to  classify those pictures as belonging to their respective dinos.


### Implementation
Implement A* path-finding algorithm FROM SCRATCH on a map in a C++ node.<br>
Your planner node must subscribe to the map and publish the path as a nav_msgs/Path on the map avoiding all the obstacles.<br>
Visualize and debug your path using Rviz.<br>
<br>
<br>

Use the attached map folder for map files. You can use map_test to test out if your planner is working before moving on to harder maps like map_maze.<br>

### Resources
Learn the A* algorithm - Tutorial1 | Tutorial2<br>
Learn about map_server<br>
Learn about the different ros2 msgs you will be using : OccupancyGrid, Path<br>
Working with RVIZ2<br>
<br><br><br>
Since a lot of you are facing issues in visualizing the map in task 7.0, here are the steps that would help you do the same:<br>
Create a ros2 workspace<br>
Clone the following repo inside the src folder : path<br>
Run the following commands in the root of your workspace:<br>
pip install catkin_pkg<br>
rosdep install --from-paths src --ignore-src -r -y<br>



### Rqt graph
![image](https://github.com/user-attachments/assets/71495329-fe4b-4df2-bfb5-8c0b916ee024)

colcon-build<br>
source install/setup.bash<br>
ros2 launch path map_launch.py<br>
<br>
Start rviz and add the /map topic which has to be visualized, you should be able to see the map, if not, keep rviz open, and rerun the map_launch.py command.<br>

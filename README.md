# Autonomous_NAV2
How to run
3 command
1. export TURTLEBOT3_MODEL=burger  
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. export TURTLEBOT3_MODEL=burger  
    ros2 run nav2_map_server map_saver_cli -f ~/map/final_project_map
3. export TURTLEBOT3_MODEL=burger  
    ./autonomous_drive.py

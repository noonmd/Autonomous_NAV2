# Autonomous_NAV2
How to run
3 command
1. export TURTLEBOT3_MODEL=burger  
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. export TURTLEBOT3_MODEL=burger  
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map/final_project_map.yaml
3. export TURTLEBOT3_MODEL=burger  
    ./autonomous_drive.py

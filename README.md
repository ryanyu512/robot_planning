# robot_planning

![crop_video_sq](https://github.com/ryanyu512/robot_planning/assets/19774686/0c187cab-36cc-465f-add7-1ffd70918d78)

This project aims at integrating ROS and global path planner to help robot navigate the environment. ROS provides a simulation environment while global path planner (such as A* and D* lite) is implemented by myself to verify my concepts. In the above demo, D* lite is utilised for robot path planning. When the robot come across some obstacles, D* could replace the path from goal to new start point based on history data. On the other hand, A* algorithm needs to compute the global plan from zero. Therefore, the computation time of D* lite could be much shorter than A* algorithm. 

Credit: The ROS environment setup and implementation details are inspired by: https://github.com/SakshayMahna/Robotics-Playground/tree/main/turtlebot3_ws



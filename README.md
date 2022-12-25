# Autopilot robot 
## The goal 
The goal of this project is to let car pass through all target points and avoid some black points without human intervention 
![small_basement](https://user-images.githubusercontent.com/86145579/209465022-77c2965c-217d-4feb-8240-0953e5d8c7d1.png)


## Installation & Run 
- Put all files into your ROS workspace 
- Build a catkin workspcae 
```bash
catkin_make
```
- Source new setup.*sh file
```bash
source devel/setup.bash
```
All environment is setted now.
- Start the simulation robot or real robot by 
```bash
 roslaunch mushr_base teleop.launch ## if utilize the real car 
 roslaunch mushr_sim teleop.launch  ## if utilize the sim car 
```
- Turn on the ParticleFilter.launch to localize the car in real environment
```
roslaunch final ParticleFilter.launch
```
- Run the ShowPath.py to show the designated route on rviz by 
```
rosrun final ShowPath.py 
```
- Visualize the car in computer by 
```
rviz 
```
- Start the car by running the Autopilot.launch by 
```
roslaunch final Autopilot.launch  
```

## Demo Video 

https://user-images.githubusercontent.com/86145579/209465313-03c73b6c-a48d-45e8-bc19-b31754870144.mp4

## Reference 
This project is based on ROS. 

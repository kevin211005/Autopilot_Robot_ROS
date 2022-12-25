#!/usr/bin/env python
### final MapServer
### mush_sim teleop  ----> remove lauch map_server 
### ParticlleFilter 
### PlannerNOde 
### rosrun final Autopilot 

import collections
from geometry_msgs.msg import  PoseStamped, PoseWithCovarianceStamped
import rospy
from nav_msgs.srv import GetMap
from final.srv import *
from LineFollower import LineFollower
from ShowPath import ShowPath
from preprocessing import read_cv
import csv 
import Utils
PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'
LINEFOLLOWER_TOPIC = '/planner_node/follower_start'
FINISH_TOPIC = '/follower/stop'

if __name__ == '__main__':
    index = int(rospy.get_param('~index', '0'))
    rospy.init_node('autopilot', anonymous=True)
    print('index')
    start = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/start.csv')[0]
    path_path = '/home/robot/catkin_ws/src/final/waypoints/real_car/path.csv'

    map_service_name = rospy.get_param("~static_map", "static_map")
    ## partical filter
    pf_topic = rospy.get_param('~particle_filter_topic', '/pf/viz/inferred_pose')
    ## line follower 
    plan_topic = rospy.get_param('~plan_topic', '/planner_node/car_plan') # default: /planner_node/car_plan
    pose_topic = rospy.get_param('~pose_topic', '/pf/viz/inferred_pose') # default: /car/pose
    plan_lookahead = rospy.get_param("~plan_lookahead", 5)# Starting val: 5
    translation_weight = rospy.get_param("~translation_weight", 1.0) # Starting val: 1.0
    service_topic = rospy.get_param("~service_topic", "/planner_node/follower_start")
    rotation_weight = rospy.get_param("~rotation_weight", 0.0)# Starting val: 0.0
    kp = rospy.get_param("~kp", 0.5)# Startinig val: 1.0
    ki = rospy.get_param("~ki", 0.05)# Starting val: 0.0
    kd = rospy.get_param("~kd", 0.3)# Starting val: 0.0
    error_buff_length = rospy.get_param("~error_buff_length", 10)# Starting val: 10
    speed = rospy.get_param("~speed", 1.0) # Default val: 1.0
    print(speed)
    rospy.loginfo(kp)
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
    print('get path')
    print('kp', kp)
    print('ki', ki)
    print('kd', kd)
    data = []
    with open(path_path) as input:
        reader = csv.reader(input)
        path = []
        for line in reader:
            if len(path) > 0 and len(line) == 1:
                data.append(path)
                path = []
            elif  len(line) == 1:
                continue
            else:
                path.append([float(line[0]), float(line[1]), float(line[2])])
        data.append(path)
    print('ready to publish path to rviz')
    print(len(data))
    sp = ShowPath(data)
    
    print('finish publish path to rviz ........')
    print('start following')
    while not rospy.is_shutdown():
        plan_msg = collections.deque()
        for i in range(0,len(data)):
            for path in data[i]:
                plan_msg.append(path)
            lf = LineFollower(plan_msg, pose_topic, plan_lookahead, translation_weight, rotation_weight,
                                kp, ki, kd, error_buff_length, speed)
            print('waitting msg...')
            rospy.wait_for_message(FINISH_TOPIC, PoseStamped)
            print(i)
        break 
    rospy.spin()

#!/usr/bin/env python

import collections
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, PoseWithCovarianceStamped
import rospy
import numpy as np
import Utils
from nav_msgs.srv import GetMap
from final.srv import *
from LineFollower import LineFollower
from preprocessing import read_cv
from LaserWanderer import generate_mpc_rollouts, LaserWanderer
PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'
LINEFOLLOWER_TOPIC = '/planner_node/follower_start'
FINISH_TOPIC = '/follower/stop'
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation' # The topic to publish controls to
## for testing 
RETURN_TOPIC = '/rollback' 
CARPOSE_TOPIC = '/car/car_pose'
if __name__ == '__main__':
    start = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/start.csv')
    target_list = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/target_waypoints.csv')
    rospy.init_node('plan_path', anonymous=True, disable_signals=True)

    map_service_name = rospy.get_param("~static_map", "static_map")

    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    index = 0
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
    source_map = start[0] if index == 0 else target_list[index]
    if index != 0:
        index += 1
    
    target_map = target_list[index] 
    source, target = Utils.map_to_world(source_map, map_info), Utils.map_to_world(target_map, map_info)

    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)
    ## line follower 
    plan_topic = rospy.get_param('~plan_topic', '/planner_node/car_plan') # default: /planner_node/car_plan
    pose_topic = rospy.get_param('~pose_topic', '/car/car_pose') # default: /car/pose
    plan_lookahead = rospy.get_param("~plan_lookahead", 5)# Starting val: 5
    translation_weight = rospy.get_param("~translation_weight", 1.0) # Starting val: 1.0
    service_topic = rospy.get_param("~service_topic", "/planner_node/follower_start")
    rotation_weight = rospy.get_param("~rotation_weight", 0.0)# Starting val: 0.0
    kp = rospy.get_param("~kp", 1.0)# Startinig val: 1.0
    ki = rospy.get_param("~ki", 3.0)# Starting val: 0.0
    kd = rospy.get_param("~kd", 3.0)# Starting val: 0.0
    error_buff_length = rospy.get_param("~error_buff_length", 10)# Starting val: 10
    speed = rospy.get_param("~speed", 1.0) # Default val: 1.0

    ### seting init pose 
    source[2] = -0.408
    pre = source
    pose_pub  = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 1)

    while not rospy.is_shutdown():
        response = str(raw_input('Please input target \n'))
        msg = response.split(" ")
        if len(msg) < 2:
            pre_pose = PoseWithCovarianceStamped()
            pre_pose.header.frame_id="map"
            pre_pose.pose.pose.position.x = pre[0]
            pre_pose.pose.pose.position.y = pre[1]
            pre_pose.pose.pose.position.z = 0
            pre_pose.pose.pose.orientation = Utils.angle_to_quaternion(pre[2])
            pose_pub.publish(pre_pose)
        else:
            pre = source
            target = [float(msg[0]), float(msg[1])] + [0.0]
            print('get_target', target)
            target = Utils.map_to_world(target, map_info)
            print(index)
            resp = get_plan(source, target)
            plan_msg = collections.deque()
            for msg in rospy.wait_for_message(plan_topic, PoseArray).poses:
                plan_msg.append([msg.position.x, msg.position.y, Utils.quaternion_to_angle(msg.orientation)])
            print('following path')
            lf = LineFollower(plan_msg, pose_topic, plan_lookahead, translation_weight, rotation_weight,
                            kp, ki, kd, error_buff_length, speed)
            rospy.wait_for_message(FINISH_TOPIC, PoseStamped)
            source = target
    rospy.spin() # Prevents node from shutting down
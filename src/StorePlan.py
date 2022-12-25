#!/usr/bin/env python
from geometry_msgs.msg import PoseArray
import rospy
import numpy as np
import Utils
from nav_msgs.srv import GetMap
from final.srv import *
from preprocessing import read_cv
import csv 
PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'
map_service_name = 'static_map'
plan_topic = '/planner_node/car_plan'
if __name__ == '__main__':
    rospy.init_node('store_path', anonymous=True, disable_signals=True)
    target_list = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/target_waypoints.csv')
    path_path = '/home/robot/catkin_ws/src/final/waypoints/real_car/path.csv'
    print('reading data')
    data = []
    count = 0 
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
                path.append(line)
        data.append(path)
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
    print("Planner node from service: ")
    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)
    print('If storing whole path press Enter\n')
    print('Storing certain path input: Index source target')
    response = str(raw_input('Please input target \n'))
    msg = str(response).split(" ")
    if len(msg) < 5:
        data = []
        for i in range(len(target_list)-1):
            source_map = [float(target_list[i][0]),float(target_list[i][1]),0.0]
            target_map = [float(target_list[i+1][0]),float(target_list[i+1][1]),0.0]
            source, target = Utils.map_to_world(source_map, map_info), Utils.map_to_world(target_map, map_info)
            resp = get_plan(source, target)
            path = []
            for msg in rospy.wait_for_message(plan_topic, PoseArray).poses:
                path.append([msg.position.x, msg.position.y, Utils.quaternion_to_angle(msg.orientation)])
            data.append(path)
    elif len(msg) == 5:
        i = int(msg[0])
        if i < len(data):
            source_map = [float(msg[1]),float(msg[2]),0.0]
            target_map = [float(msg[3]),float(msg[4]),0.0]
            source, target = Utils.map_to_world(source_map, map_info), Utils.map_to_world(target_map, map_info)
            resp = get_plan(source, target)
            path = []
            for msg in rospy.wait_for_message(plan_topic, PoseArray).poses:
                path.append([msg.position.x, msg.position.y, Utils.quaternion_to_angle(msg.orientation)])
            data[i] =path
        else: print('wrong index')
    else:
        print('wrong input')
    
    with open(path_path, 'w') as output:
        writer = csv.writer(output)
        for i,path in enumerate(data):
            writer.writerow([i])
            for pose in path:
                writer.writerow(pose)
    print('finish writing')
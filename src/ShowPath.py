#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from final.srv import *

import Utils
PATH_TOPIC = "/whole_path/car_plan" # The topic to publish controls to
CARPOSE_TOPIC = '/car/car_pose'
class ShowPath(object):
    
    def __init__(self, path):
        self.path = path
        self.plan_pub = rospy.Publisher(PATH_TOPIC, PoseArray, queue_size=1)  
        self.car_sub = rospy.Subscriber(CARPOSE_TOPIC, 
                                            PoseStamped, 
                                            self.pub,
                                            queue_size=1)
        self.pa = PoseArray()
        self.pa.header.frame_id = "/map"
        for k in range(len(path)):
            for i in xrange(len(path[k])):
                config = path[k][i]
                #   print(config)
                pose = Pose()
                pose.position.x = config[0]
                pose.position.y = config[1]
                pose.position.z = 0.0
                pose.orientation = Utils.angle_to_quaternion(config[2])
                self.pa.poses.append(pose)
    def pub(self, msg):
        self.plan_pub.publish(self.pa)

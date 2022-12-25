import Utils
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import cm
import numpy as np 
import cv2
import csv 
import rospy
# #%% read data from small_basement png 
# im_input =  mpimg.imread("/home/robot/catkin_ws/src/final/maps/small_basement.png")
# input_data  = np.array(im_input)
# imgplot = plt.imshow(input_data, cmap=cm.gray_r)
# plt.show()
# print(np.shape(input_data))
# (1236,2792) y, x 
#%% read waypoint from .csv file 
def read_cv(file__path):
    data = []
    with open(file__path) as file:
        rows = csv.reader(file, delimiter=',')
        for row in rows:
            try: 
                data.append([float(row[0]), float(row[1]), 0.0])
            except:
                continue
    return data 

# if __name__== "__main__":
    #%% block bad_waypoints position 
    # bad_point = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/bad_waypoints.csv')
    # print(bad_point)
    # map_service_name = rospy.get_param("~static_map", "static_map")
    # print("Getting map from service: ", map_service_name)
    # rospy.wait_for_service(map_service_name)
    # map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
    # for point in bad_point:
    #     x, y = int(point[1]), int(point[0])
    #     input_data[x -15:x+15,y-15: y+15] = input_data[0,0]
    # imgplot = plt.imshow(input_data, cmap=cm.gray_r)
    # cv2.imwrite("/home/robot/catkin_ws/src/final/maps/target_small_basement.pgm", input_data)
    #plt.show()
    # %% sort goog_waypoints by distanct to start_point 
    # good_point = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/good_waypoints.csv')
    # start_point = read_cv('/home/robot/catkin_ws/src/final/waypoints/real_car/start.csv')
    # midpoint = [[2617,750],[1558,953],[1292,932]]
    
    # fields = ['x','y']
    # res = [] ### [distance, point]
    # for point in good_point:
    #     res.append([((point[0] - start_point[0][0]) **2 + (point[1] - start_point[0][1]) **2)**(1/2) ,point])
    # def sortByDistance(ele):
    #     return ele[0]
    # res.sort(key = sortByDistance)
    # new_point = []
    # for i, [dis, point] in enumerate(res):
    #     new_point.append(point[0:2])
    #     if i < len(midpoint):
    #         new_point.append(midpoint[i//2])
            
    # with open('/home/robot/catkin_ws/src/final/waypoints/real_car/target_waypoints.csv', 'w') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(fields)
    #     writer.writerows(new_point)

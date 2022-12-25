import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random
import heapq

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors() # list of next nodes
    # - self.planningEnv.get_distance() # distance between two node
    # - self.planningEnv.get_heuristic() # float
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    

    while self.open:
      # get the current node, with the least f value
      # feel like not really need to use heap
      minHeap = []
      for id, c in self.open.items():
        heapq.heappush(minHeap, (c, id))
      currentNode = heapq.heappop(minHeap)
      curId, curCost = currentNode[1], currentNode[0]

      # find the goal
      if curId == self.tid:
        return self.get_solution(curId)

      # remove current node from open list
      del self.open[curId]
      # add current node to closed list
      self.closed[curId] = curCost

      # generate the children
      childrenNodes = self.planningEnv.get_successors(curId)
      # self.parent

      for childId in childrenNodes:
        # only consider the child that hasn't closed
        if childId in self.closed:
          continue

        # create f, g, h values
        curConfig = self.planningEnv.get_config(curId)
        childConfig = self.planningEnv.get_config(childId)

        if not self.planningEnv.manager.get_edge_validity(curConfig, childConfig):
          continue
        childG = self.gValues[curId] + self.planningEnv.get_distance(curId, childId)
        childH = self.planningEnv.get_heuristic(childId, self.tid)
        childF = childG + childH # purpose is to choose min f(n)

        # check if child in open list
        if childId in self.open and childG > self.gValues[curId]:
        # if childId in self.open and self.gValues[childId] > self.gValues[curId]:
          continue
        self.open[childId] = childF
        self.gValues[childId] = childG
        self.parent[childId] = curId

    return None

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  # what is the structure of plan?
  # wanna print the old and new plan
  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
      idI = random.randint(0, len(plan)-1)
      idJ = random.randint(0, len(plan)-1)
      while idI == idJ:
        idI = random.randint(0, len(plan)-1)
        idJ = random.randint(0, len(plan)-1)
      if idI > idJ:
        idJ, idI = idI, idJ
      

      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      configI = plan[idI]
      configJ = plan[idJ]
      if self.planningEnv.manager.get_edge_validity(configI, configJ):
        
        px, py, _ = self.planningEnv.manager.discretize_edge(configI, configJ)

        replacePlan = [list(item) for item in zip(px, py)]
        # flattern?
        ##newPlan = plan[:idI] + replacePlan + plan[idJ:] # combine the plan
        plan = list(plan[:idI]) + replacePlan + list(plan[idJ:]) # combine the plan
  
      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()

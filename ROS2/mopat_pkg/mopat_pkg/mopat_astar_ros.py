#Rahul D

import matplotlib.pyplot as plt
import math
import numpy as np
from threading import Thread

class Astar(Thread):

    def __init__(self, robot_index, obmap, minx, miny, maxx, maxy):
        super(Astar, self).__init__()
        self.robot_index = robot_index
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xwidth = round(maxx - minx)
        self.ywidth = round(maxy - miny)
        self.obmap = obmap
        self.motion = self.dynamics()

    class Node:
        def __init__(self, x, y, cost, previous_node):
            self.x = x
            self.y = y
            self.cost = cost
            self.previous_node = previous_node

    def set_params(self, start, goal, screen_size):
        self.r_start = (start[0], screen_size[1]-start[1])
        self.goal  = (goal[0], screen_size[1]-goal[1])

    def run(self):
        sx = self.r_start[1]
        sy = self.r_start[0]
        gx = self.goal[1]
        gy = self.goal[0]
        self.path=True
        self.plan_done = False
        start_node = self.Node(sx-self.minx,
                           sy -self.miny, 0, -1)
        goal_node = self.Node(gx- self.minx,
                          gy-self.miny, 0, -1)

        explore, Visited = dict(), dict()
        explore[self.calc_index(start_node)] = start_node
        while 1:
            if not bool(explore):
                self.path=False
                break
            c_id = min(explore, key=lambda o: explore[o].cost)
            current = explore[c_id]
            if current.x == goal_node.x and current.y == goal_node.y:
                # rospy.loginfo("LOG: A*: Robot_", self.robot_index," Found goal")
                goal_node.previous_node = current.previous_node
                goal_node.cost = current.cost
                break

            # delete it explore set
            del explore[c_id]

            # Add it to the visited set
            Visited[c_id] = current

            # searching the adjacent points
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2]+self.calc_hvalue(current,goal_node), c_id)
                node_id = self.calc_index(node)

                if node_id in Visited:
                    continue

                if self.free_node(node):
                    continue

                if node_id not in explore:
                    explore[node_id] = node
                elif explore[node_id].cost >= node.cost:
                    explore[node_id] = node

        self.px, self.py = self.calc_final_path(goal_node, Visited)
        if self.path :
        else:
            self.px = [0]
            self.py = [0]
        self.plan_done = True

    def calc_hvalue(self, node, goal):
        return max (abs(node.x - goal.x),abs(node.y - goal.y) )

    def calc_final_path(self, goal_node, Visited):
        px, py = [self.calc_position(goal_node.x, self.minx)], [
            self.calc_position(goal_node.y, self.miny)]
        previous_node = goal_node.previous_node
        while previous_node != -1:
            n = Visited[previous_node]
            px.append(self.calc_position(n.x, self.minx))
            py.append(self.calc_position(n.y, self.miny))
            previous_node = n.previous_node
        return px, py

    def calc_position(self, index, shift):
        pos = index+shift
        return pos

    def calc_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def free_node(self, node):
        qx = self.calc_position(node.x, self.minx)
        qy = self.calc_position(node.y, self.miny)
        if qx in range(self.minx,self.maxx) and qy in range(self.miny,self.maxy) and not self.obmap[node.x][node.y]:
            return False
        return True

    def dynamics(self):
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion

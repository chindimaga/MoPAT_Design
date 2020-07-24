import matplotlib.pyplot as plt
import math
import random
import numpy as np


class RFD:

    def __init__(self, obmap, minx, miny, maxx, maxy,floor_height=500, num_drops=10):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xwidth = round(maxx - minx)
        self.ywidth = round(maxy - miny)
        self.obmap = obmap
        self.motion = self.dynamics()
        self.floor_height = floor_height
        self.wall_height = 5*floor_height
        self.drops = num_drops
        self.delta = 1
        self.omega = 1
        self.alpha = 1
        self.epsilons= [0.1,0.1,0.1]#ev eu ef
        self.maxage = 10000
        self.age=0
        self.best = np.inf
        self.deposit = 100

    class Node:
        def __init__(self, x, y, path_length):
            self.x = x
            self.y = y
            self.path_length = 0
            self.sand = 0

    def find_best_route(self, sx, sy, gx, gy):
        start_node = self.Node(sx-self.minx,
                               sy - self.miny, 0, 0)
        goal_node = self.Node(gx - self.minx,
                              gy-self.miny, 0, np.inf)
        
        drops = [self.start_node.copy() for i in range(self.drops)]
        selector = [i for i in range(len(self.motion))]
        cond = True
        neighbour_nodes_x = np.zeros(len(self.motion))
        neighbour_nodes_y = neighbour_nodes_x.copy()
        grads = neighbour_nodes_x.copy()
        probs = grads.copy()
        grads_i = neighbour_nodes_x.copy()
        denom = grads.copy()

        while(self.age<self.maxage):
            erosion_count = np.ones_like(self.obmap, dtype=int)
            erosion_value = np.zeroes_like(self.obmap, dtype=float)
            self.age+=1
            for drop in drops:
                visited[self.calc_index(drop)] = 1
                visited = dict()
                while 1:
                    neighbour_nodes_x = self.motion[:, 0]+drop.x
                    neighbour_nodes_y = self.motion[:, 1]+drop.y
                    grads = self.obmap[neighbour_nodes_x,
                                       neighbour_nodes_y]-self.obmap[drop.x,drop.y]
                    check = self.obmap[neighbour_nodes_x,neighbour_nodes_y]<Wall_height
                    for i, grad in enumerate(grads):
                        if grad<0:
                            grads_i[i]=0
                            probs[i]=grad
                        elif grad>0:
                            grad_i[i]=1
                            probs[i]= self.omega/grad
                        else:
                            grads_i[i]=2
                            probs[i]=self.delta   
                        if visited(calc_index(neighbour_nodes_x[i],neighbour_nodes_y[i])):
                            probs[i]=0
                    probs = np.multiply(probs,check)
                    denom = np.power(probs,self.alpha)
                    if probs==0:
                        self.obmap[drop.x, drop.y] = min(Wall_height, self.obmap[drop.x, drop.y]+drop.sand)
                        drop.x = sx
                        drop.y = sy
                        drop.path_length=0
                        drop.sand=0
                        break 
                    j = random.choices(selector, weights=probs)
                    drop.x=neighbour_nodes_x[j]
                    drop.y = neighbour_nodes_y[j]
                    drop.path_length+=1
                    visited[self.calc_index(drop)]=1
                    erosion_count[drop.x,drop.y]+=1
                    if grads[j] < 0:
                            erosion_value += (
                                self.epsilons[grads_i[j]]*grads[i]/(drop.path_length))
                    elif grads[j]>0:
                            erosion_value += (
                                self.epsilons[grads_i[j]]/(grads[i]*drop.path_length))
                    else:
                            erosion_value += (
                                self.epsilons[grads_i[j]]/(drop.path_length))
                    if drop.x==gx and drop.y==gy:
                        if self.best>drop.path_length:
                            self.maxage=0
                            self.best =drop.path_length

            erosion_value = np.divide(erosion_value,erosion_count)
            self.obmap += self.deposit-erosion_value
            self.obmap[gs,ge]=0.0

        if self.best < np.inf:
            print("path exist")
            px,py = self.calc_final_path()
        else:
            print("path does not exist")

        return px, py

    def calc_final_path(self):
        # gradient descent(going along the steepest path to be implemented)
        return 0,0

    def calc_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)


    def dynamics(self):
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return np.array(motion)


if __name__ == "__main__":
    print("Lets use RFD!!!")

    # start and goal coordinates
    sx = -3
    sy = -5
    gx = 53
    gy = 50
    Wall_height = 1000

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 50):
        ox.append(20)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(20, 60):
        ox.append(30)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)


    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)

    floor_height = 500
    # obstacle map generation
    obmap = np.ones((xwidth+1, ywidth+1), dtype=float)*floor_height

    for iox, ioy in zip(ox, oy):
        obmap[iox-minx][ioy-miny] = Wall_height

    River = RFD(obmap, minx, miny, maxx, maxy)
    px, py = River.find_best_route(sx, sy, gx, gy)
    # print(px)
    # print(py)
    print("Done")

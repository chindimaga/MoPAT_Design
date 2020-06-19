#Robotics Club - MoPAT
#Helper file

#Import required libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib import colors

#Color Map
# 0 - while - clear cell
# 1 - black - obstacle
# 2 - red   - visited
# 3 - green - start
# 4 - yellow- destination
# 5 - blue  - path

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX

########################################################
#OpenCV Visualization

#Function to draw obstacle in white
def draw_obstacles(corner_obs, frame):
    if np.all(corner_obs != None):
        for i in range(len(corner_obs)):
            obs = np.int32(corner_obs[i][0])
            cv2.polylines(frame, [obs], True, (0,0,0), 5)
            cv2.fillConvexPoly(frame, obs, (0,0,0))

#Draw basic graph on mask
def draw_grid(x_segment, y_segment, mask):
    x_segment = x_segment
    y_segment = y_segment
    y_max, x_max, _ = np.shape(mask)
    for i in range(1, np.shape(x_segment)[0]-1):
        x_grid = x_segment[i]
        cv2.line(mask, (x_grid, 0), (x_grid, y_max), (0,0,0), 1)
    # print(x_segment)
    for i in range(1, np.shape(y_segment)[0]-1):
        y_grid = y_segment[i]
        cv2.line(mask, (0, y_grid), (x_max, y_grid), (0,0,0), 1)

########################################################
#Matplotlib Visualization

#Create grid for motion planning
def make_grid(x_segment, y_segment, r_center):
    cmap = colors.ListedColormap(['white','black','red','green','yellow'])
    x_axis = np.shape(x_segment)[0]
    y_axis = np.shape(y_segment)[0] - 2
    grid = np.ones([y_axis, x_axis])
    r_x = np.where(x_segment > r_center[0])[0][0] - 1
    r_y = np.where(y_segment > r_center[1])[0][0] - 1
    grid[r_y][r_x] = 0
    grid[r_y][r_x+1] = 2
    plt.pcolor(grid, cmap = cmap, edgecolors='k', linewidths=3)
    plt.gca().invert_yaxis()
    # plt.show()

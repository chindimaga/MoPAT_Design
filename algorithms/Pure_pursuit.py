import math as m
import matplotlib.pyplot as plt
import numpy as np


def find_lds(x_r, y_r, x_g, y_g):
    lds = (x_r-x_g)*(x_r-x_g) + (y_r-y_g)*(y_r-y_g)
    return lds

"""
returns the value of radius
"""
def calculate_curvature(x_r, y_r, phi_r, x_g, y_g):#phi_r is orientation of robot
    a = -1*np.tan(phi_r)
    b = 1
    c = x_r*np.tan(phi_r) - y_r
    x = abs(a*x_g + b*y_g + c)/m.sqrt(a*a+b*b)
    lds =  find_lds(x_r, y_r, x_g, y_g)
    if x :
        r = lds/2/x
    else:
        r = pow(10, 15)
    return r


"""
returns direction mapped to numbers
 y_lap: look ahead point same as current goal point
 uses cross product relation of robot direction vector and vector from current point to current goal point
"""
def turn_direction(phi_r, x_r,y_r,x_lap, y_lap):
    side = np.cos(phi_r)*(y_lap-y_r)-np.sin(phi_r)*(x_lap-x_r)
    #print(side)
    if(side < -pow(10,-15)):
        return -1  #right turn
    elif(side>pow(10, -15)):
        return 1  #left turn
    elif(side>-pow(10,-15) and side<0):
        return 180  # move in opposite direction
    else:
        return 0 #move in same direction


"""
returns coordinates of center of circle.
"""
def find_center(x_r, y_r, phi_r, x_g, y_g):
    a = -1*m.tan(phi_r)
    b = 1
    c = x_r*m.tan(phi_r) - y_r
    if a:
        slope = b/a
        theta = m.atan2(slope, 1.0)
    else:
        slope = 10000000000000
        theta = np.pi/2
    #print(theta*180/3.14)
    r = calculate_curvature(x_r, y_r, phi_r, x_g, y_g)
    x_c = x_r - r*m.cos(theta)
    y_c = y_r - r*m.sin(theta)
    check_r = m.sqrt((x_c-x_g)*(x_c-x_g) + (y_c-y_g)*(y_c-y_g))
    if (abs(check_r - r)< pow(10, -8)):
        return x_c, y_c
    else:
        x_c = x_r + r*m.cos(theta)
        y_c = y_r + r*m.sin(theta)
        return x_c, y_c

"""
smoothens the path to remove sharp edges
more the tolerance more the smoothness and also more the deviation from path, should
choose a optimum value
"""
def smoother(path_x=[], path_y=[], tolerance=10):
    x_new = list(path_x)
    y_new = list(path_y)
    for a in range(tolerance):
        for i in range(1, len(path_x)-1):
            x_new[i] = (x_new[i-1]+x_new[i+1])/2
            y_new[i] = (y_new[i-1]+y_new[i+1])/2
    return x_new, y_new


"""
UTILITY FUNCTIONS:

1).For simulation purpose in real situation state of the robot will
be updated using feedback from camera
"""
def update(x_r, y_r, x_g, y_g, phi_r):
    x_c, y_c = find_center(x_r, y_r, phi_r, x_g, y_g)
    slope = -1*(x_g-x_c)/(y_g-y_c)
    phi = m.atan2(slope, 1.0)
    #print(phi*180/np.pi)
    cross1 = m.cos(phi_r)*(y_r- y_c) - m.sin(phi_r)*(x_r-x_c)
    cross2 = m.cos(phi)*(y_g-y_c) - m.sin(phi)*(x_g-x_c)
    #print(cross1, cross2)
    if (cross1*cross2 >=0):
        return phi
    else:
        return phi+np.pi

def print_dir(x_r, y_r, dir):
    """
    prints dierection left or right
    """
    if (dir== -1):
        plt.text(x_r, y_r,'r', fontsize= '15')
    elif (dir == 1):
        plt.text(x_r, y_r, 'l', fontsize= '15')
    elif dir == 0:
        plt.text(x_r, y_r, 'f', fontsize= '15')
    elif dir == 180:
        plt.text(x_r, y_r, 'b', fontsize= '15')

def plot_arrow(x, y, yaw, length=10 , width=5 ,fc="b", ec="k"):
    """
    Plot arrow
    """
    plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)

def plot_circle(x_c, y_c, r, x_lim, y_lim):
    ax=plt.gca()
    circle= plt.Circle((x_c,y_c), radius= r, color='r', fill=False)
    ax.add_patch(circle)

def kill_show():
    """
    kills matplotlib window with esc key
    """
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

def plot_arena():
    """
    plots obstacle
    """
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    # for i in range(-10, 50):
    #     ox.append(20)
    #     oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(20, 60):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)

    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")


"""
Pure Pursuit controller function for two points
"""
def Pure_pursuit(x_r, y_r,phi_r, x_g, y_g):
    r = calculate_curvature(x_r, y_r, phi_r, x_g, y_g)
    x_c, y_c = find_center(x_r, y_r, phi_r, x_g, y_g)
    dir = turn_direction(phi_r, x_r,y_r,x_g, y_g)
    #plot_circle(x_c, y_c, r, 200, 200)
    #print_dir(x_r, y_r, dir)
    plot_arrow(x_r, y_r, phi_r)
    kill_show()
    plt.pause(0.1)

"""
path follower integrator function
assumes robot is at the first coordinates of the list
also needs intial angle of robot with path in radians
"""
def integrator(path_x=[], path_y=[], initial_angle=0.0):
    tolerance = 10
    x, y = smoother(path_x, path_y, tolerance)
    angle = initial_angle
    plt.plot(path_x, path_y, '-r')
    #plt.plot(x,y)
    plt.plot(x[0], y[0], "og")
    plt.plot(x[len(x)-1], y[len(y)-1], "xb")
    for i in range(0,len(x)-1):
        Pure_pursuit(x[i], y[i], angle, x[i+1], y[i+1])
        angle = update(x[i], y[i], x[i+1], y[i+1], angle)


#########################################################################################################
""" CALL INTEGRATOR """

path_x = np.arange(0, 200, 1)
path_y = [abs(x-100) for x in path_x]
angle = 7*np.pi/4

#path_y = [1 if x<100 else x-100 for x in path_x]
#angle = np.pi/1000

#path_y = [np.sin(x) for x in path_x]
#angle = np.pi/4

#path_x =[53,53,52,51,50,49,48,47,46,45,44,43,43,43,43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3]
#path_y= [50,49,48,47,46,45,44,43,42,41,40,39,38,37,36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 21, 22, 23, 24, 25, 26, 27, 28, 28, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
#angle = 5*np.pi/4
#plot_arena()

integrator(path_x, path_y, angle)
plt.show()

#########################################################################################################

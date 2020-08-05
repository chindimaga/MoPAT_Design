import cv2
import numpy as np

  """
  So the following data types are global variables I have assumed to exist, with their names and formats below
  code_bit_flag which is the code bit, time signals current value
  code_bit_flag_check this is the value of code_bit_flag when the last instace of ID() was run
  
  ***IMPORTANT*** NOTE: code_bit_flag_check is updated by the ID() function by assigning the current value of code_bit_flag before the function terminates itself
  So thee Node only has to update code_bit_flag. code_bit_flag_check is handled by ID() and need not bet touched by the node
  
  robot_code is a dictionary mapping from the robot ID to the last N bits displayed by that robot it is rotated by ID() when the next code bit is shown
  
  robot_pos is a negative indexed  dictionary mapping from the robot ID to [ X_pos, Y_pos, Theta] 
  robot_pos[0] returns a dictionary mapping from  the robot ID to [ X_pos, Y_pos, Theta] for current frame
  robot_pos[-1] returns a dictionary mapping from  the robot ID to [ X_pos, Y_pos, Theta] for previous frame
  robot_pos[-2] returns a dictionary mapping from  the robot ID to [ X_pos, Y_pos, Theta] for 2 frames ago
  This is updated by ID() after every frame 
  
  
  """

def dist(ID, robot) :


    return (  ( robot_pos[ID][0] - robot[0] )**2 +   ( robot_pos[ID][1] - robot[1] )**2 )

def rotate_robot_code() :
    n= len(robot_code[0])
    for i in robot_code.keys():
        robot_code[i].insert(0, robot_code[i].pop(n-1))




def find_robot(ID, robot_list ):
    n= len( robot_code[ID] )
    value=  robot_code[ID][n-1]
    index=-1
    d= float(inf)

    for i in range(0, len(robot_list)-1) :
        if (robot_list[i][3]==value and dist(ID, robot_list[i])<d):
            d= dist(ID, robot_list[i])
            index= i
    return [ robot_list[i][0], robot_list[i][1], robot_list[i][2] ]


def ID(robot_list) :
    global code_bit_flag, code_bit_flag_check, robot_pos, robot_code

    ID_list= robot_pos.keys()
    if ( code_bit_flag != code_bit_flag_check ):
        rotate_robot_code()

    for i in ID_list:
        robot_code[-1][i] = robot_code[0][i]
        robot_code[-2][i] = robot_code[-1][i]
        robot_code[-0][i] = find_robot(i, robot_list)









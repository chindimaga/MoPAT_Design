#Rishi

import math
import numpy as np

def dist( l1, l2):
    return math.sqrt( (l1[0] - l2[0])**2 + (l1[1] - l2[1])**2 )

def centre(l1, l2, l3):
    return (l1[0]+ l2[0]+ l3[0])/3 ,  (l1[1]+ l2[1]+ l3[1])/3

def angle(l1, l2, l3 ): # REMEMBER TO ADD THE APROPRIATE RETURN VALUE AFTER IT IS DECIDED HOW TO CALCULATE AND SET ANGLE FOR THESE ROBOTS
    return 90

def ID_bit( l1, l2, l3):
    list= [l1,l2,l3]
    multi= []
    value = []

    for x in list:
        if x[2]== "r":
            value.append(0)
        elif x[2]== "g":
            value.append(1)
        elif x[2]== "b":
            value.append(2)
        else:
            print("LED color value not r,g,b", x)



    if ( dist(l2,l3)>= dist(l3,l1)>= dist(l1,l2)):
        multi.append(9)
        multi.append(3)
        multi.append(1)

    elif (dist(l3, l2) >= dist(l2, l1) >= dist(l1, l3)):
        multi.append(9)
        multi.append(1)
        multi.append(3)

    elif (dist(l3, l1) >= dist(l2, l3) >= dist(l1, l2)):
        multi.append(3)
        multi.append(9)
        multi.append(1)

    elif (dist(l3, l1) >= dist(l2, l1) >= dist(l3, l2)):
        multi.append(1)
        multi.append(9)
        multi.append(3)

    elif (dist(l2, l1) >= dist(l3, l2) >= dist(l3, l1)):
        multi.append(3)
        multi.append(1)
        multi.append(9)

    elif (dist(l2, l1) >= dist(l3, l1) >= dist(l3, l2)):
        multi.append(1)
        multi.append(3)
        multi.append(9)

    return value[0]* multi[0] + value[1]* multi[1] + value[2]* multi[2]

def localize ( LED_list ):

    robot_list=[]

    while len(LED_list)>=3 :
        d1= float('inf')
        d2= float('inf')
        i1= 0
        i2=0

        for i in range(1, len(LED_list)-1):
            if dist( LED_list[0], LED_list[i])< d1:
                d2=d1
                i2=i1
                d1= dist( LED_list[0], LED_list[i])
                i1=i

            elif dist( LED_list[0], LED_list[i])< d2:
                d2= dist( LED_list[0], LED_list[i])

        coor1, coor2 = centre( LED_list[0], LED_list[i1], LED_list[i2] )
        robot_list.append( [ coor1, coor2, angle( LED_list[0], LED_list[i1], LED_list[i2] ), ID_bit( LED_list[0], LED_list[i1], LED_list[i2] )  ])
        LED_list.remove(LED_list[0])
        LED_list.remove(LED_list[i1])
        LED_list.remove(LED_list[i2])

    return robot_list

def main():
    pass

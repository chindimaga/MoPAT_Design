#Import required libaries
import numpy as np
import cv2
import cv2.aruco as aruco
import struct
import time
import draw_tools

#CV2 font for text
font = cv2.FONT_HERSHEY_COMPLEX

#Global mouse position
clickX = 0
clickY = 0
change = False
#Node selection
click_node = False
move = False

#Main function
def main():
    #Open the camera
    # cap = cv2.VideoCapture(1)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()
    nodes = []
    global click_node
    global move
    f = open('data.txt', 'w+')
    while(1):
        #Detect marker
        # _, frame = cap.read()
        frame = cv2.imread('frame.png')
        # frame = np.rot90(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        detected = aruco.drawDetectedMarkers(frame, corners)
        if np.all(ids != None):
            #print('Detected :',corners[0][0])
            for i in range(len(ids)):
                cv2.putText(detected, str(ids[i][0]),
                                    tuple(corners[i][0][2]),
                                    font, 0.5, (0, 0, 255), 1, 4)
        #Create mask
        mask = np.ones(frame.shape)*255
        #Draw obstacles
        draw_tools.draw_obstacles(corners, mask)
        #Create initial map
        #x_seg, y_seg = create_map(mask, robot_corners)
        #draw_tools.draw_grid(x_seg, y_seg, mask)
        #draw_tools.make_grid(x_seg, y_seg, rcenter)
        cv2.imshow("Detection", frame)
        cv2.imshow("Mask", mask)
        #End work
        if k  == ord('q'):
            f.truncate(0)
            break
    # cap.release()
    cv2.destroyAllWindows()

#Basic graph for mask guides
def create_map(mask, corners):
    width = np.linalg.norm(corners[0][0]-corners[0][1])+10
    x_segment = np.arange(0, np.shape(mask)[1], width)
    y_segment = np.arange(0, np.shape(mask)[0], width)
    x_segment = np.append(x_segment, np.shape(mask)[1])
    y_segment = np.append(y_segment, np.shape(mask)[0])
    return x_segment.astype(int), y_segment.astype(int)

if __name__ == '__main__':
    main()

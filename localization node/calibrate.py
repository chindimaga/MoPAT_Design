import cv2
import numpy as np

def nothing(x):
    pass


def calibrate():

    cam= cv2.VideoCapture(0);
    file = open(r"mask parameters", "r+")
    param_list = file.readlines()

    lh_b = int(param_list[0])
    ls_b = int(param_list[1])
    lv_b = int(param_list[2])

    uh_b = int(param_list[3])
    us_b = int(param_list[4])
    uv_b = int(param_list[5])

    lh_g = int(param_list[6])
    ls_g = int(param_list[7])
    lv_g = int(param_list[8])

    uh_g = int(param_list[9])
    us_g = int(param_list[10])
    uv_g = int(param_list[11])

    lh_r = int(param_list[12])
    ls_r = int(param_list[13])
    lv_r = int(param_list[14])

    uh_r = int(param_list[15])
    us_r = int(param_list[16])
    uv_r = int(param_list[17])

    cv2.namedWindow("Tuning for Blue Mask")
    cv2.createTrackbar("LH_B", "Tuning for Blue Mask",lh_b,255, nothing )
    cv2.createTrackbar("LS_B", "Tuning for Blue Mask",ls_b,255, nothing )
    cv2.createTrackbar("LV_B", "Tuning for Blue Mask",lv_b
                       ,255, nothing )
    cv2.createTrackbar("UH_B", "Tuning for Blue Mask",uh_b,255, nothing )
    cv2.createTrackbar("US_B", "Tuning for Blue Mask",us_b,255, nothing )
    cv2.createTrackbar("UV_B", "Tuning for Blue Mask",uv_b,255, nothing )

    while True:
        #_, frame= cam.read()                                         # importing frame
        frame= cv2.imread("test.jpg", 1)
        hsv= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    # converting from RGB to HSV colorspace

        # getting the positions of the trackbars and saving
        lh_b = cv2.getTrackbarPos("LH_B", "Tuning for Blue Mask")
        ls_b = cv2.getTrackbarPos("LS_B", "Tuning for Blue Mask")
        lv_b = cv2.getTrackbarPos("LV_B", "Tuning for Blue Mask")
        uh_b = cv2.getTrackbarPos("UH_B", "Tuning for Blue Mask")
        us_b = cv2.getTrackbarPos("US_B", "Tuning for Blue Mask")
        uv_b = cv2.getTrackbarPos("UV_B", "Tuning for Blue Mask")
 
        l_b= np.array([lh_b, ls_b, lv_b])
        u_b= np.array([uh_b, us_b, uv_b])
 
        mask = cv2.inRange(hsv, l_b, u_b)              # mask created using threshold values
        res= cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("raw feed", frame)                    # Displaying raw cam feed   # Displaying raw cam feed
        cv2.imshow("masked feed", res)  # Displaying raw cam feed
        key= cv2.waitKey(1)

        if key == 13:
            break
    cv2.destroyAllWindows()

    # Repeating tuning process for Green mask

    cv2.namedWindow("Tuning for Green Mask")
    cv2.createTrackbar("LH_G", "Tuning for Green Mask", lh_g, 255, nothing)
    cv2.createTrackbar("LS_G", "Tuning for Green Mask", ls_g, 255, nothing)
    cv2.createTrackbar("LV_G", "Tuning for Green Mask", lv_g, 255, nothing)
    cv2.createTrackbar("UH_G", "Tuning for Green Mask", uh_g, 255, nothing)
    cv2.createTrackbar("US_G", "Tuning for Green Mask", us_g, 255, nothing)
    cv2.createTrackbar("UV_G", "Tuning for Green Mask", uv_g, 255, nothing)

    while True:
        # _, frame= cam.read()                                         # importing frame
        frame = cv2.imread("test.jpg", 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # converting from RGB to HSV colorspace

        # getting the positions of the trackbars and saving
        lh_g = cv2.getTrackbarPos("LH_G", "Tuning for Green Mask")
        ls_g = cv2.getTrackbarPos("LS_G", "Tuning for Green Mask")
        lv_g = cv2.getTrackbarPos("LV_G", "Tuning for Green Mask")
        uh_g = cv2.getTrackbarPos("UH_G", "Tuning for Green Mask")
        us_g = cv2.getTrackbarPos("US_G", "Tuning for Green Mask")
        uv_g = cv2.getTrackbarPos("UV_G", "Tuning for Green Mask")

        l_g = np.array([lh_g, ls_g, lv_g])
        u_g = np.array([uh_g, us_g, uv_g])

        mask = cv2.inRange(hsv, l_g, u_g)  # mask created using threshold values
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("raw feed", frame)  # Displaying raw cam feed   # Displaying raw cam feed
        cv2.imshow("masked feed", res)  # Displaying raw cam feed
        key = cv2.waitKey(1)

        if key == 13:
            break
    cv2.destroyAllWindows()

    # Repeating tuning process for Red mask

    cv2.namedWindow("Tuning for Red Mask")
    cv2.createTrackbar("LH_R", "Tuning for Red Mask", lh_r, 255, nothing)
    cv2.createTrackbar("LS_R", "Tuning for Red Mask", ls_r, 255, nothing)
    cv2.createTrackbar("LV_R", "Tuning for Red Mask", lv_r, 255, nothing)
    cv2.createTrackbar("UH_R", "Tuning for Red Mask", uh_r, 255, nothing)
    cv2.createTrackbar("US_R", "Tuning for Red Mask", us_r, 255, nothing)
    cv2.createTrackbar("UV_R", "Tuning for Red Mask", uv_r, 255, nothing)

    while True:
        # _, frame= cam.read()                                         # importing frame
        frame = cv2.imread("test.jpg", 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # converting from RGB to HSV colorspace

        # getting the positions of the trackbars and saving
        lh_r = cv2.getTrackbarPos("LH_R", "Tuning for Red Mask")
        ls_r = cv2.getTrackbarPos("LS_R", "Tuning for Red Mask")
        lv_r = cv2.getTrackbarPos("LV_R", "Tuning for Red Mask")
        uh_r = cv2.getTrackbarPos("UH_R", "Tuning for Red Mask")
        us_r = cv2.getTrackbarPos("US_R", "Tuning for Red Mask")
        uv_r = cv2.getTrackbarPos("UV_R", "Tuning for Red Mask")

        l_r = np.array([lh_r, ls_r, lv_r])
        u_r = np.array([uh_r, us_r, uv_r])

        mask = cv2.inRange(hsv, l_r, u_r)  # mask created using threshold values
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("raw feed", frame)  # Displaying raw cam feed   # Displaying raw cam feed
        cv2.imshow("masked feed", res)  # Displaying raw cam feed
        key = cv2.waitKey(1)

        if key == 13:
            break
    cv2.destroyAllWindows()
    file.seek(0,0)
    file.writelines([str(lh_b)+"\n", str(ls_b)+"\n", str(lv_b)+"\n", str(uh_b)+"\n", str(us_b)+"\n", str(uv_b)+"\n", str(lh_g)+"\n", str(ls_g)+"\n", str(lv_g)+"\n", str(uh_g)+"\n", str(us_g)+"\n", str(uv_g)+"\n", str(lh_r)+"\n", str(ls_r)+"\n", str(lv_r)+"\n", str(uh_r)+"\n", str(us_r)+"\n", str(uv_r)])




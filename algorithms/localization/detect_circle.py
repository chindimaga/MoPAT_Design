import cv2
import numpy as np

def detect_circle(img):

    LED_list= []

    #importing mask parameters
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

    # defining the lower bound and upper bound vectors

    l_b = np.array([lh_b, ls_b, lv_b])
    u_b = np.array([uh_b, us_b, uv_b])

    l_g = np.array([lh_g, ls_g, lv_g])
    u_g = np.array([uh_g, us_g, uv_g])

    l_r = np.array([lh_r, ls_r, lv_r])
    u_r = np.array([uh_r, us_r, uv_r])

    # using these bounds on the received image to genberate a mask

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_b = cv2.inRange(img_hsv, l_b, u_b)  # mask created using threshold values
    res_b = cv2.bitwise_and(img, img, mask=mask_b)
    gray_res_b = cv2.cvtColor(res_b, cv2.COLOR_BGR2GRAY)
    gray_res_b= cv2.equalizeHist(gray_res_b)
    gray_res_b = cv2.blur(gray_res_b, (3, 3))

    cv2.imshow("masked image_b", gray_res_b)

    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_res_b,
                                        cv2.HOUGH_GRADIENT, 1, 50, param1=20,
                                        param2=10, minRadius=1, maxRadius=60)

    # Draw circles that are detected.
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            LED_list.append([pt[0],pt[1],"b", pt[2]])
            # Draw the circumference of the circle.
            cv2.circle(img, (pt[0], pt[1]), pt[2], (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (pt[0], pt[1]), 1, (0, 0, 0), 3)


   # Now detecting green circles

    mask_g = cv2.inRange(img_hsv, l_g, u_g)  # mask created using threshold values
    res_g = cv2.bitwise_and(img, img, mask=mask_g)
    gray_res_g = cv2.cvtColor(res_g, cv2.COLOR_BGR2GRAY)
    gray_res_g = cv2.equalizeHist(gray_res_g)
    gray_res_g = cv2.blur(gray_res_g, (3, 3))

    cv2.imshow("masked image_g", gray_res_g)

    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_res_g,
                                        cv2.HOUGH_GRADIENT, 1, 50, param1=20,
                                        param2=10, minRadius=1, maxRadius=60)

    # Draw circles that are detected.
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            LED_list.append([pt[0], pt[1], "g", pt[2]])
            # Draw the circumference of the circle.
            cv2.circle(img, (pt[0], pt[1]), pt[2], (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (pt[0], pt[1]), 1, (0, 0, 0), 3)

    # Now detecting red circles

    mask_r = cv2.inRange(img_hsv, l_r, u_r)  # mask created using threshold values
    res_r = cv2.bitwise_and(img, img, mask=mask_r)
    gray_res_r = cv2.cvtColor(res_r, cv2.COLOR_BGR2GRAY)
    gray_res_r = cv2.equalizeHist(gray_res_r)
    gray_res_r = cv2.blur(gray_res_r, (5, 5))

    cv2.imshow("masked image_r", gray_res_r)

    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_res_r,
                                        cv2.HOUGH_GRADIENT, 1, 60, param1=30,
                                        param2=20, minRadius=10, maxRadius=60)

    # Draw circles that are detected.
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            LED_list.append([pt[0], pt[1], "r", pt[2]])
            # Draw the circumference of the circle.
            cv2.circle(img, (pt[0], pt[1]), pt[2], (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (pt[0], pt[1]), 1, (0, 0, 0), 3)

    print(LED_list)
    cv2.imshow("Detected Circle", img)


    cv2.waitKey(0)

frame = cv2.imread("test.jpg", 1)
detect_circle(frame)

import cv2
import numpy as np

def detect_blob(img, mask_params):

    LED_list= []


    l_b = mask_params[0]
    u_b = mask_params[1]

    l_g = mask_params[2]
    u_g = mask_params[3]

    l_r = mask_params[4]
    u_r = mask_params[5]

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100

    # All to be adjusted to the real test image
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.1

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.1

    # Create a detector with the parameters
    # the if statements are in case its runs on an older version of OpenCV since the function name changes in previous versions
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # using these bounds on the received image to genberate a Blue mask

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_b = cv2.inRange(img_hsv, l_b, u_b)  # mask created using threshold values
    res_b = cv2.bitwise_and(img, img, mask=mask_b)
    gray_res_b = cv2.cvtColor(res_b, cv2.COLOR_BGR2GRAY)

    #gray_res_b= cv2.equalizeHist(gray_res_b)        HISTOGRAM EQUALIZATION LEADS TO ARTIFACTS , BUT HELPS IF CONTRAST FROM BACK GROUND IS LOW
    #gray_res_b = cv2.blur(gray_res_b, (5, 5))       MAKES FOR WEAKER EDGES  DECIDED LATER IF YOU ANT TO KEEP BOTH FOR ALL 3 COLORS

    cv2.imshow("masked image_b", 255- gray_res_b)

    # Detect blobs.
    keypoints = detector.detect(255- gray_res_b)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 0, 0),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints

    for i in range(0, len(keypoints)):
        cv2.circle(im_with_keypoints, (round(keypoints[i].pt[0]), round(keypoints[i].pt[1])), 1, (0, 0, 0), 3)
        LED_list.append([round(keypoints[i].pt[0]), round(keypoints[i].pt[1]), "b"])
    cv2.imshow("Keypoints", im_with_keypoints)

    # Repeating for green
    mask_g = cv2.inRange(img_hsv, l_g, u_g)  # mask created using threshold values
    res_g = cv2.bitwise_and(img, img, mask=mask_g)
    gray_res_g = cv2.cvtColor(res_g, cv2.COLOR_BGR2GRAY)

    #gray_res_g = cv2.equalizeHist(gray_res_g)
    #gray_res_g = cv2.blur(gray_res_g, (5, 5))

    cv2.imshow("masked image_g", 255 - gray_res_g)

    # Detect blobs.
    keypoints = detector.detect(255 - gray_res_g)

    # Draw detected blobs as black circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, np.array([]), (0, 0, 0),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints

    for i in range(0, len(keypoints)):
        cv2.circle(im_with_keypoints, (round(keypoints[i].pt[0]), round(keypoints[i].pt[1])), 1, (0, 0, 0), 3)
        LED_list.append([round(keypoints[i].pt[0]), round(keypoints[i].pt[1]), "g"])
    cv2.imshow("Keypoints", im_with_keypoints)

    # Repeating for Red


    mask_r = cv2.inRange(img_hsv, l_r, u_r)  # mask created using threshold values
    res_r = cv2.bitwise_and(img, img, mask=mask_r)
    gray_res_r = cv2.cvtColor(res_r, cv2.COLOR_BGR2GRAY)

    #gray_res_r = cv2.equalizeHist(gray_res_r)
    #gray_res_r = cv2.blur(gray_res_r, (5, 5))

    cv2.imshow("masked image_r", 255 - gray_res_r)

    # Detect blobs.
    keypoints = detector.detect(255 - gray_res_r)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, np.array([]), (0, 0, 0),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints

    for i in range(0, len(keypoints)):
        cv2.circle(im_with_keypoints, (round(keypoints[i].pt[0]), round(keypoints[i].pt[1])), 1, (0, 0, 0), 3)
        LED_list.append([round(keypoints[i].pt[0]), round(keypoints[i].pt[1]), "r"])
    cv2.imshow("Keypoints", im_with_keypoints)



  #  print(LED_list)
    cv2.waitKey(0)
    return LED_list


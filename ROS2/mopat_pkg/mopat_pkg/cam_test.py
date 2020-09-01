import cv2

cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(5, 30)
cap.set(16, 2)
# a = cap.set(cv2., 64)
# print(a)
# cap.set(32, 1)
# cap.set(10, 0)
# for i in [9,10,11,12,14,16,32]:
#     a = cap.get(i)
#     print(a)

def brightness(val):
    cap.set(10, val)
def contrast(val):
    cap.set(11, val)
def saturation(val):
    cap.set(12, val)
def sharp(val):
    cap.set(20, val)


cv2.namedWindow("sliders")
cv2.createTrackbar("B", "sliders", 255, 255, brightness)
cv2.createTrackbar("C", "sliders", 32, 255, contrast)
cv2.createTrackbar("S", "sliders", 32, 255, saturation)
cv2.createTrackbar("Sh", "sliders", 64, 255, sharp)

while 1:


    _, frame = cap.read()
    cv2.imshow("sliders", frame)
    if cv2.waitKey(1) == ord('q'): break

cap.release()
# cv2.destroyAllWindows()

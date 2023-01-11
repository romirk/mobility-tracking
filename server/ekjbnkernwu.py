import cv2

from broker import detect

frame = cv2.imread("test.jpg")
result = detect(frame)
print(result)

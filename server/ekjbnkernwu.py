import cv2

from broker import detect

frame = cv2.imread("test.jpg")
print(frame)
result = detect(frame)
print(result)

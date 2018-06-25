import cv2
import numpy


img = cv2.imread('map1.png',0)

img_buffer1 = cv2.imread('map1.png',0)
img_buffer2 = cv2.imread('map1.png',0)
img_buffer3 = cv2.imread('map1.png',0)
img_buffer4 = cv2.imread('map1.png',0)
img_buffer5 = cv2.imread('map1.png',0)
img_buffer6 = cv2.imread('map1.png',0)
img_buffer7 = cv2.imread('map1.png',0)
img_buffer8 = cv2.imread('map1.png',0)
img_buffer9 = cv2.imread('map1.png',0)
img_buffer10 = cv2.imread('map1.png',0)
img_buffer11 = cv2.imread('map1.png',0)
img_buffer12 = cv2.imread('map1.png',0)
img_buffer13 = cv2.imread('map1.png',0)

buffer=[img_buffer1,img_buffer2,img_buffer3,img_buffer4,img_buffer5,img_buffer6,img_buffer7,img_buffer8,img_buffer9,img_buffer10,img_buffer11,img_buffer12,img_buffer13]

for k in range(12):
    for y in range(1,len(img)-1):
        for x in range(1,len(img[y])-1):
            if buffer[k][y,x]==255 and (buffer[k][y,x+1]==0 or buffer[k][y+1,x+1]==0 or buffer[k][y+1,x]==0 or buffer[k][y+1,x-1]==0 or buffer[k][y,x-1]==0 or buffer[k][y-1,x-1]==0 or buffer[k][y-1,x]==0 or buffer[k][y-1,x+1]==0):
                buffer[k+1][y,x]=0


cv2.imshow('original',img)
cv2.imshow('buffer',img_buffer13)
cv2.waitKey(0)
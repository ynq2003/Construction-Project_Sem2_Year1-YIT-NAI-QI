import cv2
import numpy as np
import imutils

cap= cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

def empty(a):
    pass

cv2.namedWindow('HSV')
cv2.resizeWindow('HSV',640,240)
cv2.createTrackbar('hue min','HSV',0,179,empty)
cv2.createTrackbar('hue max','HSV',179,179,empty)
cv2.createTrackbar('sat min','HSV',0,255,empty)
cv2.createTrackbar('sat max','HSV',255,255,empty)
cv2.createTrackbar('v min','HSV',0,255,empty)
cv2.createTrackbar('v max','HSV',255,255,empty)


while True:
    
    _,img= cap.read()

    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    h_min=cv2.getTrackbarPos('hue min','HSV')
    h_max=cv2.getTrackbarPos('hue max','HSV')
    s_min=cv2.getTrackbarPos('sat min','HSV')
    s_max=cv2.getTrackbarPos('sat max','HSV')
    v_min=cv2.getTrackbarPos('v min','HSV')
    v_max=cv2.getTrackbarPos('v max','HSV')
    
    lower_red = np.array([156,136,0])
    upper_red = np.array([179,255,255])
    lower_green = np.array([58,94,15])
    upper_green = np.array([70,255,255])
    lower_yellow = np.array([31,75,105])
    upper_yellow = np.array([55,255,255])
    lower_blue = np.array([100,130,0])
    upper_blue = np.array([179,255,255])
    lower_black = np.array([0,0,0])
    upper_black = np.array([179,255,74])
    
    lower= np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask= mask)
    
    mask=cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    hstack=np.hstack([mask,result])
    
    #cv2.imshow('original',img)
    cv2.imshow('hstack',hstack)
    #cv2.imshow('mask',mask)
    #cv2.imshow('result',result)
    
    k = cv2.waitKey(5)
    if k == 27:
        break
    
cap.release()
cv2.destroyAllWindows()
import cv2 
import numpy as np

cap = cv2.VideoCapture(1)

while(1):

    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([98,117,124])   #array([hMin,sMin,vMin])
    upper_blue = np.array([206,240,255]) #array([hMax,sMax,vMax])
    ''' 
    lower_red = np.array([170,50,50]) #example value
    upper_red = np.array([180,255,255]) #example value
    lower_green = np.array([50,50,50]) #example value
    upper_green = np.array([70,255,255]) #example value
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    res_red = cv2.bitwise_and(frame,frame, mask= mask_red)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    res_green = cv2.bitwise_and(frame,frame, mask= mask_green)
    '''
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    res_blue = cv2.bitwise_and(frame,frame, mask= mask_blue)

    #cv2.imshow('frame',frame)
    cv2.imshow('res_blue',res_blue)
    #cv2.imshow('res_red',res_red)
    #cv2.imshow('res_green',res_green)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()

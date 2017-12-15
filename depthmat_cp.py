import numpy as np
import cv2
import cv
import freenect
import numpy as np
import time
import math


range_dic=((400,677,50),(677,724,100),(724,834,150),(834,890,200))

def getDepthMat():
   
    depth,timestamp = freenect.sync_get_depth()
        
    depth = depth * np.logical_and(depth > 500, depth < 1024)
    #depth=depth*0.2480544747081712

    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
    
    depth = depth.astype(np.uint8)
    #edges = cv2.Canny(depth, threshold1=100, threshold2=100)
    

    depth = cv2.medianBlur(depth,17)
    depth = cv2.bilateralFilter(depth,9,75,75)    
    frame=depth
    laplacian = cv2.Laplacian(frame,cv2.CV_64F)
   
  
    #cv2.imshow('Canny',edges)
    cv2.imshow('Original',frame)
    cv2.imshow('laplacian',laplacian)
    

    return depth
'''
while True:
    depth = getDepthMat()
    print(depth)
    cv2.imshow('Depth', depth)
    cv2.waitKey(10)
'''
i=0
while True:
    i=i+1    
    depth=getDepthMat()
    print(depth)    
    cv2.imshow('depth',depth)
    cv2.waitKey(1)



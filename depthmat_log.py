import numpy as np
import cv2
import cv
import freenect
import numpy as np
import time
import math

data = []
range_dic=((400,677,50),(677,724,100),(724,834,150),(834,890,200))

def getDepthMat():
   
    depth,timestamp = freenect.sync_get_depth()
    depth=cv2.resize(depth, (360, 240))
        
    #depth = depth * np.logical_and(depth > 500, depth < 1024)
    #depth=depth*0.2480544747081712

    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
    
    #depth = depth.astype(np.uint8)
    #edges = cv2.Canny(depth, threshold1=100, threshold2=100)
    

    depth = cv2.medianBlur(depth,5)
    #Pdepth = cv2.bilateralFilter(depth,9,75,75)    
    #frame=depth
    #
   
  
    #cv2.imshow('Canny',edges)
    #cv2.imshow('Original',depth)
    #cv2.imshow('laplacian',laplacian)
    

    return np.array(depth)
'''
while True:
    depth = getDepthMat()
    print(depth)
    cv2.imshow('Depth', depth)
    cv2.waitKey(10)
'''

while True:
    
    depth=getDepthMat()
    cv2.imshow('Depth_O',depth) 
   
    sd= np.std(depth)
    m = np.mean(depth)
    for i in range(depth.shape[0]):
        for j in range(depth.shape[1]):
            #depth[i,j]=(abs(depth[i,j]-m)/sd)*255
            depth[i,j]=((math.log((256-depth[i,j]),255))**3)*255
    equ = cv2.equalizeHist(depth)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl1 = clahe.apply(depth)
    cv2.imshow('cl1',cl1)

    res = np.hstack((equ,depth))
    '''
    for ii in ((0,60),(60,120),(120,180),(180,241)):
        for jj in ((0,45),(45,90),(90,135),(135,181)):
            try:
                mat=np.matrix(depth[ii[0]:ii[1],jj[0]:jj[1]])
                #m=mat.max()
                sd= np.std(mat)
                m = np.mean(mat)
                
            except:
                pass

            print(m)
            for i in range(ii[0],ii[1]-1):
                for j in range(jj[0],jj[1]-1):
                    print(i,j)
                    try:
                        depth[i,j]=(abs(depth[i,j]-m)/sd)*255
                        #depth[i,j]=math.log((256-depth[i,j]),255)*255
                        #depth[i,j]=((math.log((m-depth[i,j]),m))**1.2)*(m-1)
                    except:
                        pass
    '''
    #laplacian = cv2.Laplacian(depth,cv2.CV_64F)           
    #print(depth)    
    print(depth.shape)
    cv2.imshow('depth',res)
   # cv2.imshow('laplacian',laplacian)
    cv2.waitKey(1)











import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep

mtx = np.fromfile('/home/pi/comp_vis/mtx.npy').reshape(3,3)
dist = np.fromfile('/home/pi/comp_vis/dist.npy')
width = 512
height = 288
#width = 1024
#height = 576
camera = PiCamera()
camera.resolution = (width,height)
camera.framerate = 30
camera.iso = 100
sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains 
camera.awb_mode = 'off'
camera.awb_gains = g

lowColor = np.array([96,105,65],np.uint8)
upColor = np.array([135,255,200],np.uint8)
kernel = np.ones((5,5),np.uint8)

def constant():
    camera.capture('/home/pi/comp_vis/Assignment2/constant.jpg')
    img=cv.imread('/home/pi/comp_vis/Assignment2/constant.jpg')
    h,w=img.shape[:2]
    ncm,roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    new = cv.undistort(img,mtx,dist,None,ncm)
    x,y,w,h = roi
    img2=new[y:y+h,x:x+w]
    global wi
    global hi
    hi,wi=img2.shape[:2]
    cv.imwrite('/home/pi/comp_vis/new.jpg',img2)
    
    
def work(img2):
    hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv,lowColor,upColor)
    final = cv.bitwise_and(img2,img2, mask = mask)
    kernel = np.ones((5,5),np.uint8)
    clean3 = cv.blur(final,(3,3))
#   clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
#   clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
    grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(grey,15,255,cv.THRESH_BINARY)
    img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    return img0,contours,heirarchy

def end(img0):
    pixel = 0
    mea = 0
    drive = 0
    found= False
    for i in range(hi):
            for j in range(wi):
                if img0[i][j] > 0:
                    pixel = i
                    print("pixel: %d" % pixel)
                    found = True
                    break
            if found:
                break
                    
    if pixel < 48:
        drive = 18
        mea = 100
    elif (pixel >48 )and(pixel<=59):
        drive = (30-18+6*(59-pixel)/(59-48))
        mea = drive+18
    elif (pixel >59 )and(pixel<=74):
        drive = (24-18+6*(74-pixel)/(74-59))
        mea = drive+18
    elif (pixel >74 )and(pixel<99):
        num = (18-18+6*(99-pixel)/(99-74))
        mea = num+18
        if mea<21:
            drive = mea
        else:
            drive = num
            
    elif(pixel >= 99)and(pixel<=117):
        drive = (15+3*(117-pixel)/(117-99))
        mea = drive
    elif(pixel > 117):
        drive = (12+3*(144-pixel)/(144-117))
        mea = drive
        
        #48 = 36in
        #59 = 30in
        #74 = 24in
        #99= 18in
        #117= 15in
        #144= 12in
    return drive,mea
def begin(img0):
    pixel = 0
    mea = 0
    drive = 0
    found= False
    for i in range(hi-1,0,-1):
            for j in range(wi):
                if img0[i][j] > 0:
                    pixel = i
                    print("pixel: %d" % pixel)
                    found = True
                    break
            if found:
                break
                    
    if pixel < 48:
        drive = 18
        mea = 100
    elif (pixel >48 )and(pixel<=59):
        drive = (30-18+6*(59-pixel)/(59-48))
        mea = drive+18
    elif (pixel >59 )and(pixel<=74):
        drive = (24-18+6*(74-pixel)/(74-59))
        mea = drive+18
    elif (pixel >74 )and(pixel<99):
        num = (18-18+6*(99-pixel)/(99-74))
        mea = num+18
        if mea<21:
            drive = mea
        else:
            drive = num
            
    elif(pixel >= 99)and(pixel<=117):
        drive = (15+3*(117-pixel)/(117-99))
        mea = drive
    elif(pixel > 117):
        drive = (12+3*(144-pixel)/(144-117))
        mea = drive
    return drive,mea

def ang(contours):
    angle = 0
    try:
        cnt_c = contours[0]
        M_c = cv.moments(cnt_c)
    
        cxc = int(M_c['m10']/M_c['m00'])
        cyc = int(M_c['m01']/M_c['m00'])
        fov = 62.2
        fudge = 1.00
        if(cxc<(wi/2)): 
            angle = (fov/2)*fudge*(((wi/2)-cxc)/((wi/2)))
            print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
        else:
            angle = -(fov/2)*((cxc-(wi/2))/(wi/2))
            print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
    except ZeroDivisionError:
        angle = 0
    except IndexError:
        print("No markers found.")
#       no tape
    return angle
def pic():
    constant()
    img2=cv.imread('/home/pi/comp_vis/new.jpg')
    img0,contours,heirarchy = work(img2)
    return img0,contours
    
def main():
    angle = 0
    toggle = 0
    img0,contours=pic()
    #pic takes one picture
    drive,mea=end(img0)
    drive,mea=begin(img0)
    #end finds end of tape
    #begin finds start of tape
    angle = ang(contours)
    #angle finds angle
    #probably want a while loop with pic for angle state
    
    print("drive: {:.2f}" .format(drive))
    print("distane to tape: {:.2f}" .format(mea))
#    cv.imshow("pic",img0)
#    cv.waitKey(0)
#    cv.destroyAllWindows
        
    
while(True):
    main()
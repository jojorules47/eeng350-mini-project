import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep
import Demo2Comms as comms

mtx = np.fromfile('/home/pi/comp_vis/mtx.npy').reshape(3,3)
dist = np.fromfile('/home/pi/comp_vis/dist.npy')
width = 512
height = 288
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
    hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv,lowColor,upColor)
    final = cv.bitwise_and(img2,img2, mask = mask)
    kernel = np.ones((5,5),np.uint8)
    clean3 = cv.blur(final,(3,3))
#    clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
#    clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
    grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(grey,15,255,cv.THRESH_BINARY)
    img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    return img0,contours,heirarchy

def pic():
    img0,contours,heirarchy = constant()
    return img0,contours

def ang(contours):
    angle = 0
    try:
        cnt_c = contours[0]
        M_c = cv.moments(cnt_c)
    
        cxc = int(M_c['m10']/M_c['m00'])
        cyc = int(M_c['m01']/M_c['m00'])
        #tape true
        comms.tape = True
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
        comms.tape = True
    except IndexError:
        print("No markers found.")
        comms.tape = False
    return angle

def run():
    while(True):
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
        hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv,lowColor,upColor)
        final = cv.bitwise_and(img2,img2, mask = mask)
        kernel = np.ones((5,5),np.uint8)
        clean3 = cv.blur(final,(3,3))
#    clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
#    clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
        grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(grey,15,255,cv.THRESH_BINARY)
        img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        angle = 0
        try:
            cnt_c = contours[0]
            M_c = cv.moments(cnt_c)
    
            cxc = int(M_c['m10']/M_c['m00'])
            cyc = int(M_c['m01']/M_c['m00'])
            #tape true
            comms.tape = True
            fov = 62.2
            fudge = 1.00
            if(cxc<(wi/2)): 
                angle = -(fov/2)*fudge*(((wi/2)-cxc)/((wi/2)))
                print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
            else:
                angle = (fov/2)*((cxc-(wi/2))/(wi/2))
                print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
        except ZeroDivisionError:
            angle = 0
            comms.tape = True
        except IndexError:
            print("No markers found.")
            comms.tape = False
        comms.angle = angle
        comms.writeNumber(1, comms.tape)
    



def find_tape():
    print("Starting Robot")
    turn = int(input("right = 1 left = 0: (0/1)"))
    if (turn):
        print("right")
        comms.angle = 360
    else:
        print("left")
        comms.angle = -360
    
    comms.writeNumber(3, comms.tape)
    comms.angle = 90
    print("Finding tape")
    #print("newstate: ", state) # find
    #find tape and rotate to within 5 deg then its go time
    while abs(comms.angle) > 5:
        img0,contours0 = pic()
        angle0 = ang(contours0)
        if comms.tape == True: break
    comms.angle = angle0
    #break loop when tape true
    comms.writeNumber(0, comms.tape)
    
def go():
    print("Following Line")
    run()

        
find_tape()
go()
        
        
        
        
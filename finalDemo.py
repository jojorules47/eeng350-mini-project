import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep
import Demo2Comms as comms

mtx = np.fromfile('/home/pi/comp_vis/mtx.npy').reshape(3,3)
dist = np.fromfile('/home/pi/comp_vis/dist.npy')
#width = 512
#height = 288
width = 288
height = 144
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
    img2=cv.imread('/home/pi/comp_vis/Assignment2/constant.jpg')
#    h,w=img.shape[:2]
#    ncm,roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
#    new = cv.undistort(img,mtx,dist,None,ncm)
#    x,y,w,h = roi
#    img2=new[y:y+h,x:x+w]
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
        cross = False
        camera.capture('/home/pi/comp_vis/Assignment2/constant.jpg')
        img2=cv.imread('/home/pi/comp_vis/Assignment2/constant.jpg')
#        h,w=img.shape[:2]
#        ncm,roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
#        new = cv.undistort(img,mtx,dist,None,ncm)
#        x,y,w,h = roi
#        img2=new[y:y+h,x:x+w]
        global wi
        global hi
        hi,wi=img2.shape[:2]
        hsv = cv.cvtColor(img2,cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv,lowColor,upColor)
        final = cv.bitwise_and(img2,img2, mask = mask)
        kernel = np.ones((5,5),np.uint8)
        clean = cv.blur(final,(3,3))
        clean2 = cv.morphologyEx(clean, cv.MORPH_OPEN, kernel)
        clean3 = cv.morphologyEx(clean2, cv.MORPH_CLOSE, kernel)
        grey = cv.cvtColor(clean3,cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(grey,15,255,cv.THRESH_BINARY)
        img0,contours,heirarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        angle = 0
#        found = False
#        i = 0
#        cv.imshow("pic",img0)
#        cv.waitKey(0)
#        cv.destroyAllWindows
        for contour in contours:
            approx = cv.approxPolyDP(contour, 0.05*cv.arcLength(contour, True), True)
            print("points", len(approx))
            if len(approx) > 5 and len(approx) < 7:
                cross = True
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
            angle = 45
            flag = True
        flag = False
        
        comms.angle = angle
        if comms.angle > 10 and flag == False: comms.angle = 10
        elif comms.angle < -10 and flag == False: comms.angle = -10
        if cross == True:
            comms.distance = 10
        
        comms.writeNumber(1, comms.tape)
        
        if cross == True:
            break
    



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
    comms.angle = 6
    
    print("Finding tape")
    #print("newstate: ", state) # find
    #find tape and rotate to within 5 deg then its go time
    while abs(comms.angle) > 5:
        img0,contours0 = pic()
        angle0 = ang(contours0)
        if comms.tape == True: break
    comms.angle = angle0
    if comms.angle > 10: comms.angle = 10
    elif comms.angle<-10: comms.angle = -10
    #break loop when tape true
    comms.writeNumber(0, comms.tape)
    
def go():
    print("Following Line")
    run()

        
find_tape()
go()
print("done!")
#while (True):
#    test,contours = pic()
#    cv.imshow("pic",test)
#    
#    i=0
#    for contour in contours:
#            approx = cv.approxPolyDP(contour, 0.05*cv.arcLength(contour, True), True)
#            print("points", len(approx))
#            if len(approx) > 10 and len(approx) < 13:
#                print("cross")
#    cv.waitKey(0)
#    cv.destroyAllWindows()
            
        
        
        
        
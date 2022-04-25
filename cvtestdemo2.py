#this needs to run at startup
import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep

import Demo2Comms as comms

mtx = np.fromfile('/home/pi/comp_vis/mtx.npy').reshape(3,3)
dist = np.fromfile('/home/pi/comp_vis/dist.npy')
width = 512
height = 288

sl = 0.5
sle = .7
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

#end startup code

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
    
    
def work(img2):
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

def end(img0):
    pixel = 0
    mea = 100
    drive = 18
    mid = wi/2
    found= False
    for i in range(hi):
            for j in range(wi):
                if img0[i][j] > 0:
                    pixel = i
                    mid = j
                    print("pixel: %d" % pixel)
                    found = True
                    break
            if found:
                break
                
            # <=?
    if pixel < 48:
        drive = 18
        mea = 100
        print("over 36 away, driving: %d" % (drive))
    elif (pixel >48 )and(pixel<=59):
        drive = (30-18+6*(59-pixel)/(59-48))
        mea = drive+18
        print("distance: %d, driving: %d" % (mea, drive))
    elif (pixel >59 )and(pixel<=74):
        drive = (24-18+6*(74-pixel)/(74-59))
        mea = drive+18
        print("distance: %d, driving: %d" % (mea, drive))
    elif (pixel >74 )and(pixel<99):
        num = (18-18+6*(99-pixel)/(99-74))
        mea = num+18
        if mea<19:
            drive = mea
            print("distance: %d, driving: %d" % (mea, drive))
        else:
            drive = num
            print("distance: %d, driving: %d" % (mea, drive))
            
    elif(pixel >= 99)and(pixel<=117):
        drive = (15+3*(117-pixel)/(117-99))
        mea = drive
        print("distance: %d, driving: %d" % (mea, drive))
    elif(pixel > 117):
        drive = (12+3*(144-pixel)/(144-117))
        mea = drive
        print("distance: %d, driving: %d" % (mea, drive))
    fov = 62.2
    fudge = 1.00
    if(mid<(wi/2)): 
        angle = (fov/2)*fudge*(((wi/2)-mid)/((wi/2)))
        print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
    else:
        angle = -(fov/2)*((mid-(wi/2))/(wi/2))
        print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
        
        #48 = 36in
        #59 = 30in
        #74 = 24in
        #99= 18in
        #117= 15in
        #144= 12in
    return drive,mea,angle


def begin(img0):
    pixel = 0
    mea = 100
    drive = 18
    mid = wi/2
    found= False
    for i in range(hi-1,0,-1):
            for j in range(wi):
                if img0[i][j] > 0:
                    pixel = i
                    mid = j
                    print("pixel: %d" % pixel)
                    found = True
                    break
            if found:
                break
                    
    if pixel < 48:
        drive = 18
        print("over 36 away, drive: %d" % drive)
        mea = 100
    elif (pixel >48 )and(pixel<=59):
        drive = (30-18+6*(59-pixel)/(59-48))
        mea = drive+18
        print("distance: %d, driving: %d" % (mea, drive))
    elif (pixel >59 )and(pixel<=74):
        drive = (24-18+6*(74-pixel)/(74-59))
        mea = drive+18
        print("distance: %d, driving: %d" % (mea, drive))
    elif (pixel >74 )and(pixel<99):
        
        num = (18-18+6*(99-pixel)/(99-74))
        mea = num+18
        if mea<19:
            drive = mea
            print("distance: %d, driving: %d" % (mea, drive))
        else:
            drive = num
            print("distance: %d, driving: %d" % (mea, drive))
            
    elif(pixel >= 99)and(pixel<=117):
        drive = (15+3*(117-pixel)/(117-99))
        mea = drive
        print("distance: %d, driving: %d" % (mea, drive))
    elif(pixel > 117):
        drive = (12+3*(144-pixel)/(144-117))
        mea = drive
        print("distance: %d, driving: %d" % (mea, drive))
    fov = 62.2
    fudge = 1.00
    if(mid<(wi/2)): 
        angle = (fov/2)*fudge*(((wi/2)-mid)/((wi/2)))
        print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
    else:
        angle = -(fov/2)*((mid-(wi/2))/(wi/2))
        print("The camera needs to turn {:.2f} degrees to center the marker." .format(angle))
    return drive,mea,angle

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
def pic():
    constant()
    img2=cv.imread('/home/pi/comp_vis/new.jpg')
    img0,contours,heirarchy = work(img2)
    return img0,contours
    
    #main is trash
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
    
    
###### Begin worker functions ######
def find_tape():
    #state machine
#    state = comms.none
#    #while no tape:
#    img0,contours0 = pic()
#    angle0 = ang(contours0)
    #break loop when tape true
    
    print("Starting Robot")
    #sleep(0.5)
    #print("state: ", state) # none
    #state = state(comms.tape)
    turn = int(input("l? (0/1)"))
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
    while not comms.tape:
        img0,contours0 = pic()
        angle0 = ang(contours0)
    comms.angle = angle0
    #break loop when tape true
    comms.writeNumber(0, comms.tape)
    

    #time.sleep(1)
    #state = state(comms.tape)
    #print("newstate: ", state) # find
    #print("Finding tape")
    #time.sleep(1)
    #tape = True
    #print("Stopping Robot")
    #state = state(tape) # return none
    #end loop
    
def rotate():
    ack = False
    #once tape is found
#    comms.angle = 10

    #while comms.angle > 2:
    #img1,contours1 = pic()
    #comms.angle = -1.0*ang(contours1)
    #print("Rotating the robot", comms.angle)
    
        #read angle 1
        #rotate to angle
    print("test1")
    while abs(comms.angle) > 1:
        print("test2")
#        input('Do a rotate?')
        img1,contours1 = pic()
#        cv.imshow('pic', img1)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        coma = -1*ang(contours1)
        comms.angle = coma
        print("Off by", comms.angle)
        
        comms.writeNumber(2, comms.tape)
        ##### wait for aurduino acknowdledge #####
        
        while (ack == False):
            ack = comms.readNumber()
#            print("Waiting...")
            sleep(sle)
        ack = False
        print("Acknowledged!")
    
    #sleep(5) # Wait for arduino to rotate. May replace
    #check tolerance
    #img2,contours2 = pic()
    #angle2 = ang(contours2)
    #check if in tolerance, if not, do this again

def drive_to_start():
    ack = False
    #x
    drive = 0
    mea = 100
    while True:
#        input('Drive?')
        sleep(sl)
        img3,contours3 = pic()
#        cv.imshow('pic', img3)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
        sleep(sl)
        drive,mea,angle=begin(img3)
        if abs(angle) > 1:
#            comms.angle = angle
            img1,contours1 = pic()
#        cv.imshow('pic', img1)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
            coma = -1.0*(ang(contours1))
            comms.angle = coma 
            print("Off by", comms.angle)
#            input('Do a rotate?')
            comms.writeNumber(2, comms.tape)
        ##### wait for aurduino acknowdledge #####
            while (ack == False):
                ack = comms.readNumber()
#                print("Waiting...")
                sleep(sle)
            ack = False
#            print("Acknowledged!")
#            sleep(1)
        comms.distance = drive
        print("Driving the robot to start", comms.distance)
        comms.writeNumber(1, comms.tape)
        
        while (ack == False):
            ack = comms.readNumber()
#            print("Waiting...")
            sleep(sle)
        ack = False
        
        if mea == drive:
            break
        if not comms.tape:
            break

        ##### wait for aurduino acknowdledge #####
        
#        print("Acknowledged!")
        
#        sleep(10) # Wait for arduino to stop
    #drive distance "drive"
    #if mea == drive: done
    # else: go back to x and run again until done

def drive_to_end():
    ack = False
    drive = 0
    mea = 100
    while True:
#        input('Drive?')
        sleep(sl)
        img4,contours4 = pic()
        sleep(sl)
        drive,mea,angle=end(img4)
        if abs(angle) > 1:
#            comms.angle = angle
            img1,contours1 = pic()
#        cv.imshow('pic', img1)
#        cv.waitKey(0)
#        cv.destroyAllWindows()
            coma = -1.0*(ang(contours1))
            comms.angle = coma
            print("Off by", comms.angle)
#            input('Do a rotate?')
            comms.writeNumber(2, comms.tape)
        ##### wait for arduino acknowdledge #####
            while (ack == False):
                ack = comms.readNumber()
                print("Waiting...")
                sleep(sle)
            ack = False
            print("Acknowledged!")
#            sleep(1.5)
        comms.distance = drive
        print("Driving the robot to end", comms.distance)
        comms.writeNumber(1, comms.tape)
        
        while (ack == False):
            ack = comms.readNumber()
            print("Waiting...")
            sleep(sle)
        ack = False
        print("Acknowledged!")
        
        print("MEA:", mea, "DRIVE", drive)
        if mea == drive:
            break

        ##### wait for aurduino acknowdledge #####
        
#        sleep(10) # Wait for arduino to stop



if __name__ == '__main__':
    test2 = True
    
    find_tape()
#    input("Rotate?")
    rotate()
#    input("Drive?")
    drive_to_start()
    #input("End?")
    sleep(1)
    if test2:
        drive_to_end()


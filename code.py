import cv2
import numpy as np        
import RPi.GPIO as GPIO        #calling for header file which helps in using GPIOs of P
import time
video = cv2.VideoCapture(0)     
video1 = cv2.VideoCapture(1)
GPIO.setmode(GPIO.BCM)     #programming the GPIO by BCM pin numbers. (like PIN40 as GPIO21)
GPIO.setwarnings(False)

#ultrasonic
TRIG = 27
ECHO = 22
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)

#led
GPIO.setup(5, GPIO.OUT)
GPIO.output(5, True)

Left_motor_forward = 26
Left_motor_backward = 19
Right_motor_forward = 24
Right_motor_backward = 23

GPIO.setup(Left_motor_forward, GPIO.OUT)
GPIO.setup(Left_motor_backward, GPIO.OUT)
GPIO.setup(Right_motor_forward, GPIO.OUT)
GPIO.setup(Right_motor_backward,GPIO.OUT)
pwm_lmf = GPIO.PWM(Left_motor_forward,400)
pwm_rmf = GPIO.PWM(Right_motor_forward,400)
pwm_lmb = GPIO.PWM(Left_motor_backward,400)
pwm_rmb = GPIO.PWM(Right_motor_backward,400)


pwm_lmf.start(0)
pwm_rmf.start(0)
pwm_lmb.start(0)
pwm_rmb.start(0)

def stop():
    pwm_lmf.ChangeDutyCycle(0)
    pwm_rmf.ChangeDutyCycle(0)
    pwm_lmb.ChangeDutyCycle(0)
    pwm_rmb.ChangeDutyCycle(0)
    
def forward(LR, RR, USP):
    pwm_lmf.ChangeDutyCycle(LR * speed * USP)
    pwm_rmf.ChangeDutyCycle(RR * speed * USP)
    pwm_lmb.ChangeDutyCycle(0)
    pwm_rmb.ChangeDutyCycle(0)

speed = 50
keep_running = True


font = cv2.FONT_HERSHEY_SIMPLEX

distance = 100
while keep_running:

    ret, orig_frame = video.read()
    ret, orig_frame = video.read()
    ret, orig_frame = video.read()
    ret, orig_frame = video.read()
    ret, orig_frame = video.read()
    ret, orig_frame = video.read()
    

    frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_blue = np.array([110, 100, 50])
    up_blue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, low_blue, up_blue)
    edges = cv2.Canny(mask, 75, 150)
 
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20, maxLineGap=50)
    if lines is not None:
        for line in lines[0]:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1, y1), (x2, y2), (0, 240, 0), 5)
 
    
    verPix = 170
    horPix = 200
    camDistance = "none"
    for i in range(0, 639, 1):
        if (frame[horPix][i][1] == 240 and frame[horPix][i][0] == 0):
            camDistance = i
            break
        
    cv2.line(frame, (verPix, 0), (verPix, 480), (255, 0, 0), 5)
    cv2.line(frame, (0, horPix), (640, horPix), (255, 0, 255), 3)

    
    # front camera
    cirRad = 0
    currentTS = "green"
    ret,TLimg = video1.read()
    ret,TLimg = video1.read()
    ret,TLimg = video1.read()
    ret,TLimg = video1.read()
    ret,TLimg = video1.read()  
    gray = cv2.cvtColor(TLimg, cv2.COLOR_BGR2GRAY)
    TLimg1 = cv2.medianBlur(gray,5)
    circles = cv2.HoughCircles(TLimg1,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=175,param2=35,minRadius=0,maxRadius=70)
    currentTS = "none"
    if circles is not None:
        circles = np.uint16(np.around(circles))
        counter=0
        correctC=[]
        xC=[]
        yC=[]
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(TLimg,(i[0],i[1]),i[2],(0,255,0),2)#cimg
            correctC.append((i[0],i[1],i[2]))
            xC.append(i[0])
            yC.append(i[1])
            counter+=1
        cirRad = circles[0][0][2]
        xCS=sorted(xC)
        yCS=sorted(yC)
        xS=sorted(correctC, key=lambda correctC:correctC[0])
        q1=sorted(xS[:4],key=lambda correctC: correctC[1])
        
        if counter == 3:
            if TLimg1[yCS[0],xCS[0]] > TLimg1[yCS[1],xCS[1]] and TLimg1[yCS[0],xCS[0]] > TLimg1[yCS[2],xCS[2]]:
                currentTS = "red"
            elif TLimg1[yCS[1],xCS[1]] >  TLimg1[yCS[0],xCS[0]] and TLimg1[yCS[1],xCS[1]] >TLimg1[yCS[2],xCS[2]]:
                currentTS = "yellow"
            else:
                currentTS = "green"


    if currentTS != "none":
        if currentTS == "red":
            color = (0, 0, 255)
            cv2.putText(TLimg, "STOP!", (50, 250), font, 4, color, 7, 1)
        elif currentTS == "yellow":
            color = (0, 255, 255)
        elif currentTS == "green":
            color = (0, 255, 0)
            cv2.putText(TLimg, "GO!", (50, 250), font, 4, color, 7, 1)
        cv2.putText(TLimg, currentTS, (50, 100), font, 4, color, 7, 1)
            


    #ultrasonic
    lastDis = distance
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
    if distance > lastDis + 20:
        distance = distance + 20
    if (distance <= 20):
        USP = 0
    elif(distance > 20 and distance <= 100):
        USP = (distance - 20.0) * 0.0125
    else:
        USP = 1

    fullPix = 100.0
    RR = 0
    LR = 0
    if (camDistance == "none" or ((currentTS == "red" or currentTS == "yellow") and cirRad >= 25) or USP == 0):
        stop()
        print 'stop 1'
        time.sleep(1)
    else:
        if camDistance >= verPix:
            rightDis = camDistance - verPix
            if rightDis > fullPix:
                rightDis = fullPix
                speed = 100
            else:
                speed = 50
            LR = 1.00 - rightDis / fullPix
            RR = 1
        elif camDistance < verPix:
            LeftDis = verPix - camDistance
            if LeftDis > fullPix:
                LeftDis = fullPix
                speed = 100
            else:
                speed = 50
            RR = 1.00 - LeftDis / fullPix
            LR = 1
        forward(RR, LR, USP)
    print ('distance:', distance,"  camDistance:", camDistance," currentTS:",currentTS) 
    print(" cirRad:",cirRad, ' RR:', RR, ' LR:', LR, ' USP:', USP, ' speed:' , speed)
    cv2.imshow("lines", frame)
    cv2.imshow('traffic',TLimg)
    if cv2.waitKey(10) == 27:
        keep_running = False
        print "out"
        
video.release()
video1.release()
video2.release()
cv2.destroyAllWindows()
GPIO.cleanup()

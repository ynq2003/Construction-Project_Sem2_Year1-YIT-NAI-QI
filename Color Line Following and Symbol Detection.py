import cv2
import numpy as np
import imutils
import RPi.GPIO as GPIO
from time import sleep
import os
import VideoStream
import time

## Camera settings
IM_WIDTH = 320
IM_HEIGHT = 240 
FRAME_RATE = 10

videostream = VideoStream.VideoStream((IM_WIDTH,IM_HEIGHT),FRAME_RATE,2,0).start()
time.sleep(1) # Give the camera time to warm up

img1="Traffic Light"
img2="Stop"

flag=0

class PID:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        # Skip integral term calculation if Ki is zero
        if self.Ki != 0:
            self.integral += error

            # Anti-windup (limiting integral term)
            self.integral = max(self.min_output / self.Ki, min(self.max_output / self.Ki, self.integral))

        derivative = error - self.prev_error
        self.prev_error = error

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Constrain output within range of min_output to max_output
        output = max(self.min_output, min(self.max_output, output))
        return output

in1 = 17
in2 = 27
in3 = 22
in4 = 23

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

# Create a PWM object for controlling the speed
# Set up hardware PWM
pwm1 = GPIO.PWM(in1, 100)  # Pin, Frequency
pwm2 = GPIO.PWM(in2, 100)  # Pin, Frequency
pwm3 = GPIO.PWM(in3, 100)  # Pin, Frequency
pwm4 = GPIO.PWM(in4, 100)  # Pin, Frequency

#Color threshold
lower_red = np.array([161,107,0])
upper_red = np.array([179,255,255])
     
lower_green = np.array([75,165,61])
upper_green = np.array([79,255,99])

lower_yellow = np.array([20,100,76])
upper_yellow = np.array([33,255,255])

lower_blue = np.array([81,70,60])
upper_blue = np.array([130,255,255])

lower_black = np.array([0,0,0])
upper_black = np.array([179,255,54])

def shapeDetection(image):
    cv2.imshow("Frame1",image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Perform edge detection
    edges = cv2.Canny(blur, 50, 150)
    # Find contours
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Classifying shapes
    for contour in contours:
        epsilon = 0.025 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)
        area = cv2.contourArea(contour)
        
        if area > 800:
            if vertices == 3:
                shape = "Triangle"
                print(shape)
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                return shape
            elif vertices == 4:
                shape = "Rectangle"
                print(shape)
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                return shape
            elif vertices == 5:
                shape = "Pentagon"
                print(shape)
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                return shape
            elif vertices == 6:
                shape = "Hexagon"
                print(shape)
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
            elif vertices == 8:
                (x, y), (w, h), angle = cv2.fitEllipse(contour)  # Fit ellipse to contour
                circularity = cv2.contourArea(contour) / (np.pi * (w / 2) * (h / 2))  # Calculate circularity
                #print(circularity)
                if circularity < 0.99:  # Threshold for circularity
                    shape = "Partial Circle"
                    print(shape)
                    cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                    cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                    return shape
                else:
                    shape = "Circle"
                    print(shape)
                    cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                    cv2.putText(image, shape, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                    return shape
                      
            else:
                shape = ""
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # Calculate angle between centroid and tip of the arrow
                    angle = np.arctan2(approx[0][0][1] - cY, approx[0][0][0] - cX) * 180 / np.pi
                    #print(angle)
                    
                    if angle < 0:
                        angle += 360
                        # Determine direction based on angle
                    if 260 < angle <= 280:
                        direction = "Up"
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                        cv2.putText(image, direction, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                    elif 135 < angle <= 225:
                        direction = "Left"
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                        cv2.putText(image, direction, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                    elif 235 < angle <= 255:
                        direction = "Down"
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                        cv2.putText(image, direction, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                    else:
                        direction = "Right"
                        cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                        cv2.putText(image, direction, (approx[0][0][0], approx[0][0][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0), 2)
                shape = direction
                # Put text at the centroid
                print(shape)
                return shape
                flag=1
                return flag
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                
      
min_output = -80  # Minimum PWM value
max_output = 80   # Maximum PWM value
# Parameters for the PID controller
Kp = 0.6
Ki = 0.0
Kd = 0.0

pid = PID(Kp, Ki, Kd, min_output, max_output)

#def stop(frame1):
 #   time.sleep(0.1)
  #  frame1=videostream.read()
   # shapeDetection(frame1)
    #pwm1.start(0)
    #pwm2.start(0)
    #pwm3.start(0)
    #pwm4.start(0)
    #time.sleep(0.8)
    #frame1=videostream.read()
    #shapeDetection(frame1)
    #sleep(2)
    #frame1=videostream.read()
    #shapeDetection(frame1)
    #time.sleep(1)

while True:
    frame= videostream.read()
    frame=frame[130:240,0:320]
    cv2.imshow("Frame",frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame1 = videostream.read()
    shapeDetection(frame1)

    if cv2.waitKey(10) & 0xFF == ord('a'):
        pwm1.start(0)
        pwm2.start(0)
        pwm3.start(0)
        pwm4.start(0)
        for i in range(10):
            print(img2)
        
        time.sleep(2)
    
    # Perform color thresholding for each color
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    # Find contours for each color mask
    red_contours = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours= cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours= cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_contours= cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_contours= cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     
    red_contours = imutils.grab_contours(red_contours)
    yellow_contours = imutils.grab_contours(yellow_contours)
    blue_contours = imutils.grab_contours(blue_contours)
    green_contours = imutils.grab_contours(green_contours)
    black_contours = imutils.grab_contours(black_contours)
    
    if len(red_contours)>0:
        c = max(red_contours, key=cv2.contourArea)
        cv2.drawContours(frame, c, -1, (0,255,0), 1)
        M = cv2.moments(c)
        if M["m00"] !=0 :
           cx = int(M['m10']/M['m00'])
           #cy = int(M['m01']/M['m00'])
           #print('red',"CX : "+str(cx))
           
           error = cx - 160  # Calculate error (difference between centroid and center of the frame)

           # Use PID controller to adjust motor speeds based on the error
           correction = pid.update(error)
           #print(correction)
            
           left_speed = 20 + correction
           right_speed = 20 - correction
           Tleft_speed = 25 + correction
           Tright_speed = 25 - correction
           
           uplimit = 55
           dolimit = 0
           Tuplimit = 80
           Tdolimit = 0
           
           if left_speed > uplimit:
               left_speed = uplimit
           elif left_speed < dolimit:
               left_speed = dolimit
           if right_speed > uplimit:
               right_speed = uplimit
           elif right_speed < dolimit:
               right_speed = dolimit
           if Tleft_speed > Tuplimit:
               Tleft_speed = Tuplimit
           elif Tleft_speed < Tdolimit:
               Tleft_speed = Tdolimit
           if Tright_speed > Tuplimit:
               Tright_speed = Tuplimit
           elif Tright_speed < Tdolimit:
               Tright_speed = Tdolimit
               
           if cx >= 283 :
                #print("Turn Right")
                pwm1.start(Tleft_speed)
                pwm2.start(0)
                pwm3.start(0)
                pwm4.start(left_speed) 
           elif cx < 283 and cx >37 :
                #print("On Track!")
                pwm1.start(left_speed)
                pwm2.start(0)
                pwm3.start(right_speed)
                pwm4.start(0)
           elif cx <=37:
                #print("Turn Left")
                pwm1.start(0)
                pwm2.start(right_speed)
                pwm3.start(Tright_speed)
                pwm4.start(0)
                
    elif len(blue_contours)>0:
        c = max(blue_contours, key=cv2.contourArea)
        cv2.drawContours(frame, c, -1, (0,255,0), 1)
        M = cv2.moments(c)
        if M["m00"] !=0 :
           cx = int(M['m10']/M['m00'])
           #cy = int(M['m01']/M['m00'])
           #print('blue',"CX : "+str(cx))
           
           error = cx - 160  # Calculate error (difference between centroid and center of the frame)

           # Use PID controller to adjust motor speeds based on the error
           correction = pid.update(error)
           #print(correction)
            
           left_speed = 20 + correction
           right_speed = 20 - correction
           Tleft_speed = 25 + correction
           Tright_speed = 25 - correction
           
           uplimit = 55
           dolimit = 0
           Tuplimit = 80
           Tdolimit = 0
           
           if left_speed > uplimit:
               left_speed = uplimit
           elif left_speed < dolimit:
               left_speed = dolimit
           if right_speed > uplimit:
               right_speed = uplimit
           elif right_speed < dolimit:
               right_speed = dolimit
           if Tleft_speed > Tuplimit:
               Tleft_speed = Tuplimit
           elif Tleft_speed < Tdolimit:
               Tleft_speed = Tdolimit
           if Tright_speed > Tuplimit:
               Tright_speed = Tuplimit
           elif Tright_speed < Tdolimit:
               Tright_speed = Tdolimit
               
           if cx >= 283 :
                #print("Turn Right")
                pwm1.start(Tleft_speed)
                pwm2.start(0)
                pwm3.start(0)
                pwm4.start(left_speed) 
           elif cx < 283 and cx >37 :
                #print("On Track!")
                pwm1.start(left_speed)
                pwm2.start(0)
                pwm3.start(right_speed)
                pwm4.start(0)
           elif cx <=37:
                #print("Turn Left")
                pwm1.start(0)
                pwm2.start(right_speed)
                pwm3.start(Tright_speed)
                pwm4.start(0)
              
    elif len(green_contours)>0:
        #print('green')
        #stop(frame1)
        cv2.imshow("Frame1",frame1)
        gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Perform edge detection
        edges = cv2.Canny(blur, 50, 150)
        # Find contours
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
          # Classifying shapes
        for contour in contours:
            epsilon = 0.025 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            vertices = len(approx)
            area = cv2.contourArea(contour)
            #print("vertice",vertices)
        
            if area > 800:
                if vertices >6:
                    shape = ""
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        # Calculate angle between centroid and tip of the arrow
                        angle = np.arctan2(approx[0][0][1] - cY, approx[0][0][0] - cX) * 180 / np.pi
                        #print(angle)
                    
                        if angle < 0:
                            angle += 360
                        # Determine direction based on angle
                        if 260 < angle <= 280:
                            direction = "Up"
                        elif 135 < angle <= 225:
                            direction = "Left"
                        elif 235 < angle <= 255:
                            direction = "Down"
                        else:
                            direction = "Right"
                        shape = direction
                        # Put text at the centroid
                        print(shape)          
           
                
    elif len(black_contours)>0:
        #print('red')
        c = max(black_contours, key=cv2.contourArea)
        cv2.drawContours(frame, c, -1, (0,255,0), 1)
        M = cv2.moments(c)
        if M["m00"] !=0 :
           cx = int(M['m10']/M['m00'])
           #cy = int(M['m01']/M['m00'])
           #print('black',"CX : "+str(cx))

           error = cx - 160  # Calculate error (difference between centroid and center of the frame)

           # Use PID controller to adjust motor speeds based on the error
           correction = pid.update(error)
           #print(correction)
            
           left_speed = 20 + correction
           right_speed = 20 - correction
           Tleft_speed = 25 + correction
           Tright_speed = 25 - correction
           
           uplimit = 55
           dolimit = 0
           Tuplimit = 80
           Tdolimit = 0
           
           if left_speed > uplimit:
               left_speed = uplimit
           elif left_speed < dolimit:
               left_speed = dolimit
           if right_speed > uplimit:
               right_speed = uplimit
           elif right_speed < dolimit:
               right_speed = dolimit
           if Tleft_speed > Tuplimit:
               Tleft_speed = Tuplimit
           elif Tleft_speed < Tdolimit:
               Tleft_speed = Tdolimit
           if Tright_speed > Tuplimit:
               Tright_speed = Tuplimit
           elif Tright_speed < Tdolimit:
               Tright_speed = Tdolimit
               
           if cx >= 283 :
                #print("Turn Right")
                pwm1.start(Tleft_speed)
                pwm2.start(0)
                pwm3.start(0)
                pwm4.start(left_speed) 
           elif cx < 283 and cx >37 :
                #print("On Track!")
                pwm1.start(left_speed)
                pwm2.start(0)
                pwm3.start(right_speed)
                pwm4.start(0)
           elif cx <=37:
                #print("Turn Left")
                pwm1.start(0)
                pwm2.start(right_speed)
                pwm3.start(Tright_speed)
                pwm4.start(0)
           else :
                print("I don't see the line")
                pwm1.start(0)
                pwm2.start(0)
                pwm3.start(0)
                pwm4.start(0)
                
    #else:         
        #print("I dont see the line")
        #pwm1.start(0)
        #pwm2.start(25)
        #pwm3.start(0)
        #pwm4.start(25)




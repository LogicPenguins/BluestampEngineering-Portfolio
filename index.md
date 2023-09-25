# Session 2: Custom Ball Tracking Robot
Voice-controlled Autonomous Computer Vision Robot with Object Classification, 180 servo based robotic Arm and a tracking + dictionary build-in module

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ahanaf Z | The Browning School | Astronautical Engineering | Incoming Junior

# T'is I
![face](https://user-images.githubusercontent.com/88998860/180488838-6e9b1647-cd90-4da9-84f6-4e29d74626b2.jpg)


# Final Session 2 Robot
![finalrobot](https://user-images.githubusercontent.com/88998860/180488857-468eb9c4-2e26-42b3-bbb5-aa9f2c4e01d9.jpg)


# Fifth Milestone
My fifth and final milestone wraps up my time at Bluestamp and all the awesome things I've built and learned while I've attended the program. The final list of mechanisms that my robot is able to perform are: 
- Robot movement 
- Computer Vision-backed object tracking 
- 5 180-degree-servo control for picking up objects 
- Urban Dictionary Looker-Upper through audio input 
- Voice control commands and call/response outputs 
- Text to speech and speech to text 
- Manipulation of the OLED screen 
- Tensorflow model for object detection and classification

I go more indepth on how each of these mechanisms operate in the Youtube video linked below so I won't explain those in this short write up. However being able to create a robot that I tried to make as close to a multipurpose rover as I could, really helped me improve my vision on one day working at an aerospace company like SpaceX or NASA and finally helping to further human understanding in the final frontier: Space. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/Aj4xxKJ7XbM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Fourth Milestone
My fourth milestone was kind of like the start of a new chapter for me on my Bluestamp journey. This marked the beginnings of my more "high-end" Adeept Pi-Car Pro kit. It came in with a bunch of built in modules but since I wanted this robot to be as custom as possible, I ended up writing most of the software from scratch by referencing the documentation for the libraries Adeept used in their sample code, but without looking at their code implementation. I was able to get a number of components working by my fourth milestone as shown in the Youtube video below, like the OLED, the servos, and the transfer of the ball-tracking code from my 2 wheel chassi of my first session to the much nicer 4 wheel tank drive with a servo in the front to control direction on this new Adeept robot. The instructions weren't the best in my opinion, as I had spent round 4-5 hours building the base robot alone, but once I finished, it was without a doubt worth it! 

<iframe width="560" height="315" src="https://www.youtube.com/embed/8s561L8bf9g" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Session 2 Code

## Main Code
```py
import time
import RPi.GPIO as GPIO
import cv2
import os
import argparse
import numpy as np
import importlib.util
from picamera2 import Picamera2
import pyttsx3
from adafruit_servokit import ServoKit
from pynput.keyboard import Listener
import pyttsx3
import speech_recognition as sr
import sys
from playsound import playsound
from pydub import AudioSegment
from pydub.playback import play
import threading


speed_set_forward = 50
speed_set_other = 60
kit = ServoKit(channels=16)

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

servo0_degree = 90
servo1_degree = 90
servo2_degree = 90
servo3_degree = 90
servo4_degree = 90

def motorStop():#Motor stops
    print('Motor stopping')
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    try:
        pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        pwm_B = GPIO.PWM(Motor_B_EN, 1000)
    except:
        pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
    if status == 0: # stop
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
    if status == 0: # stop
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
    else:
        if direction == Dir_forward:#
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
    return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1  
    #speed = 100
    if direction == 'forward':
        if turn == 'right':
            kit.servo[0].angle = 0
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
        else:
            kit.servo[0].angle = 90
            motor_left(1, left_forward, speed)
            motor_right(1, right_forward, speed)
    elif direction == 'backward':
        if turn == 'right':
            kit.servo[0].angle = 180
            motor_left(0, left_forward, int(speed*radius))
            motor_right(1, right_backward, speed)
        elif turn == 'left':
            kit.servo[0].angle = 0
            motor_left(1, left_backward, speed)
            motor_right(0, right_forward, int(speed*radius))
        else:
            kit.servo[0].angle = 90
            motor_left(1, left_backward, speed)
            motor_right(1, right_backward, speed)
    elif direction == 'no':
        if turn == 'right':
            kit.servo[0].angle = 0
            motor_left(1, left_backward, speed)
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            kit.servo[0].angle = 180
            motor_left(1, left_forward, speed)
            motor_right(1, right_backward, speed)
        else:
            motorStop()
    else:
        pass

def destroy():
    motorStop()

def find_blob(blob):
    largest_contour = 0
    cont_index = 0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour = area

            cont_index = idx
    
    r = (0, 0, 2, 2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r, largest_contour    

def show(key):
        global servo0_degree 
        global servo1_degree 
        global servo2_degree 
        global servo3_degree
        global servo4_degree
        try:

                if key.char == 'q':
                        servo0_degree += 5
                        kit.servo[0].angle = servo0_degree
                elif key.char == 'a':
                        servo0_degree -=5
                        kit.servo[0].angle = servo0_degree
                elif key.char == 'w':
                        servo1_degree +=5
                        kit.servo[1].angle = servo1_degree
                elif key.char == 's':
                        servo1_degree -=5
                        kit.servo[1].angle = servo1_degree
                elif key.char == 'e':
                        servo2_degree +=5
                        kit.servo[2].angle = servo2_degree
                elif key.char == 'd':
                        servo2_degree -=5
                        kit.servo[2].angle = servo2_degree
                elif key.char == 'r':
                        servo3_degree +=5
                        kit.servo[3].angle = servo3_degree
                elif key.char == 'f':
                        servo3_degree -=5
                        kit.servo[3].angle = servo3_degree 
                elif key.char == 't':
                        servo4_degree +=5
                        kit.servo[4].angle = servo4_degree
                elif key.char == 'g':
                        servo4_degree -=5
                        kit.servo[4].angle = servo4_degree 
                elif key.char == 'z':
                        return False
                
                # Collect all event until released
        except ValueError:
                
                if servo0_degree < 20 or servo0_degree > 160:
                        servo0_degree = 90
                        kit.servo[0].angle = servo0_degree
                if servo1_degree < 20 or servo1_degree > 160:
                        servo1_degree = 90
                        kit.servo[1].angle = servo1_degree
                if servo2_degree < 20 or servo2_degree > 160:
                        servo2_degree = 90
                        kit.servo[2].angle = servo2_degree
                if servo3_degree < 20 or servo3_degree > 160:
                        servo3_degree = 90
                        kit.servo[3].angle = servo3_degree
                if servo4_degree < 20 or servo4_degree > 160:
                        servo4_degree = 90
                        kit.servo[4].angle = servo4_degree

def get_audio():
        r = sr.Recognizer()
        said = ''
        while True:
            with sr.Microphone() as source:
                    audio = r.listen(source)


                    try:
                            said = r.recognize_google(audio)
                            
                            if said == 'stop':
                                print('Program terminating...')
                                sys.exit()
                            print(said)
                    except Exception as e:
                            print('Exception: ' + str(e))
                    
            if said == '':
                print('waiting for audio.')
                continue
            else:
                break
                        
        return said

def play_audio():
    print('song begun')
    song = AudioSegment.from_mp3('oldtown.mp3')
    play(song)


def dance():
    for i in range(20):
        print(i)
        move(speed_set_other, 'forward', 'right', 0.8)
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'backwards', 'right', 0.8)
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'forward', 'left', 0.8)
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'forward', 'right', 0.8)
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'forward', 'right', 0.8) 
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'forward', 'no', 0.8)    
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)
        move(speed_set_other, 'backwards', 'no', 0.8) 
        time.sleep(0.2)
        destroy()
        time.sleep(0.01)


# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

cv2.startWindowThread()

picam2 = Picamera2()

config = picam2.preview_configuration(main={"size": (640, 480), "format": "RGB888"})

picam2.configure(config)

picam2.start()

setup()

# Initialize video stream
time.sleep(1)

count = 0
#for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

text_speech = pyttsx3.init()

text_speech.setProperty('rate', 180)

robot_modes = [1, 2]

while True:
    robot_mode = int(input('Which mode would you like to activate? (1 for Autonomous, 2 for Voice-Controlled): '))

    if robot_mode not in robot_modes:
        print('Invalid mode. Please choose either 1 or 2.')
        continue
    else:
        break

if robot_mode == 1:
    while True:
        try:
            print('--------------')
            count+=1
            t1 = cv2.getTickCount()

            frame1 = picam2.capture_array()
        
            hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
            
            lower_range = np.array([120,150,50])
            upper_range = np.array([255,255,255])
            
            mask = cv2.inRange(hsv, lower_range, upper_range)

            cv2.imshow('mask',mask)

            centre_x = 0.
            centre_y = 0.


            frame = frame1.copy()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (width, height))
            input_data = np.expand_dims(frame_resized, axis=0)

            if floating_model:
                input_data = (np.float32(input_data) - input_mean) / input_std
            
            interpreter.set_tensor(input_details[0]['index'],input_data)
            interpreter.invoke()

            # Retrieve detection results
            boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
            classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
            scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
            #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)
            # Loop over all detections and draw detection box if confidence is above minimum threshold
            for i in range(len(scores)):
                if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                    # Get bounding box coordinates and draw box
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    ymin = int(max(1,(boxes[i][0] * imH)))
                    xmin = int(max(1,(boxes[i][1] * imW)))
                    ymax = int(min(imH,(boxes[i][2] * imH)))
                    xmax = int(min(imW,(boxes[i][3] * imW)))
                    
                    cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                    # Draw label
                    object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                    text_speech.say(f'I see a {object_name}')
                    text_speech.runAndWait()
                    label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                    label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                    cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                    cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

            # Draw framerate in corner of frame
            cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
            # All the results have been drawn on the frame, so it's time to display it.
            cv2.imshow('Object detector', frame)

            # Calculate framerate
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc= 1/time1

            loct, area = find_blob(mask)

            x, y, w, h = loct

            if (w*h) < 600:
                found=0
            else:
                found=1
                simg2 = cv2.rectangle(frame1, (x,y), (x+w,y+h), 255,2)
                centre_x=x+((w)/2)
                centre_y=y+((h)/2)
                cv2.circle(frame1,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
                centre_x-=80
                centre_y=6--centre_y
            initial=300
            flag=0

            if(found==0):
                print('Found 0 activated')

                if flag==0:
                    print('Turning right.')
                    move(speed_set_other, 'forward', 'right', 0.8)
                    time.sleep(0.3)
                else:
                    print('Turning left')
                    move(speed_set_other, 'forward', 'left', 0.8)
                    time.sleep(0.3)
                destroy()
                time.sleep(0.0125)
            
            elif(found==1):
                print('Found 1 activated')
                print('area: ' + str(area))
                
                if(area>=initial):
                    initial2=30000
                    if(area<initial2):
                        move(speed_set_forward, 'forward', 'no', 0.8)
                        time.sleep(0.6)
                        destroy()
                        print('About to destroy')
                        time.sleep(0.1)
                    else:
                        with Listener(on_press = show) as listener:   
                                listener.join()
                                print('Terminating...')


        except KeyboardInterrupt:
            print('Turning off.')
            destroy()
            GPIO.cleanup()
            break

elif robot_mode == 2:
    text_speech = pyttsx3.init()

    text_speech.setProperty('rate', 140)

    pre_texts = {
            'what is your name': 'My name is Spirit Junior. But my creator, Ahanaf Zaman, says I am destined for greater things.',
            'what is the best thing about Switzerland': 'I do not know, but the flag is a big plus.',
            'why dont scientists trust atoms': 'Because they make up everything.',
    }

    dance_commands = ['dance', 'can you dance', 'please dance', 'dance for me', 'dance dog', 'why dont you dance?', 'go ahead, dance', 'do you know how to dance', 'show me some moves', 'show me how to dance']


    while True:
        my_text = get_audio().lower()

        if my_text in pre_texts.keys():
            text_speech.say(pre_texts[my_text])
            text_speech.runAndWait()
            continue
        elif my_text in dance_commands:
            a = threading.Thread(target=play_audio)
            d = threading.Thread(target=dance)

            a.start()
            d.start()

cv2.destroyAllWindows()
```

## Text to Speech Code
```py
import pyttsx3
import speech_recognition as sr
import sys
import requests, sys, bs4, re

def get_audio():
        r = sr.Recognizer()
        with sr.Microphone() as source:
                audio = r.listen(source)
                said = ""

                try:
                        said = r.recognize_google(audio)
                        
                        if said == 'stop':
                            print('Program terminating...')
                            sys.exit()
                        print(said)
                except Exception as e:
                        print('Exception: ' + str(e))
        return said

text_speech = pyttsx3.init()

text_speech.setProperty('rate', 140)

pre_texts = {
        'rikky': 'Stinky. Quite stinky really.'
}


while True:
        my_text = get_audio().lower()

        if my_text in pre_texts.keys():
                text_speech.say(pre_texts[my_text])
                text_speech.runAndWait()
                continue
        else:
                res = requests.get('https://www.urbandictionary.com/define.php?term=' + my_text)
                res.raise_for_status()

                soup = bs4.BeautifulSoup(res.text, 'html.parser')

                definition = soup.find('div', attrs = {'class':'definition'}) 

                meaning = str(definition.find_all('div', attrs = {'class':'meaning'}))
                example = str(definition.find_all('div', attrs = {'class':'example'}))

                CLEANR = re.compile('<.*?>') 

                clean_meaning = re.sub(CLEANR, '', meaning)
                clean_example = re.sub(CLEANR, '', example)

                text_speech.say(clean_meaning)
                text_speech.runAndWait()

                text_speech.say(clean_example)
                text_speech.runAndWait()

```

## OLED Code

```py
#!/usr/bin/env/python3
# File name   : server.py
# Description : for OLED functions
# Website	 : www.gewbot.com
# Author	  : William(Based on Adrian Rosebrock's OpenCV code on pyimagesearch.com)
# Date		: 2019/08/28

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1325, ssd1331, sh1106
import time
import threading

try:
	serial = i2c(port=1, address=0x3C)
	device = ssd1306(serial, rotate=0)
except:
	print('OLED disconnected\nOLED没有连接')

# Box and text rendered in portrait mode
# with canvas(device) as draw:
# 	draw.text((0, 0), "WWW.CODELECTRON.COM", fill="white")
# 	draw.text((0, 10), "WWW.CODELECTRON.COM", fill="white")
# 	draw.text((0, 20), "WWW.CODELECTRON.COM", fill="white")
# 	draw.text((0, 30), "WWW.CODELECTRON.COM", fill="white")
# 	draw.text((0, 40), "WWW.CODELECTRON.COM", fill="white")
# 	draw.text((0, 50), "WWW.CODELECTRON.COM", fill="white")
# while 1:
# 	time.sleep(1)

text_1 = 'Hello'
text_2 = 'Im Ahanaf'
text_3 = 'Whats your name?'
text_4 = 'this is a new robot'
text_5 = 'Pretty cool huh?'
text_6 = 'okay bye'

class OLED_ctrl(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(OLED_ctrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()	 # 用于暂停线程的标识
		self.__flag.set()	   # 设置为True
		self.__running = threading.Event()	  # 用于停止线程的标识
		self.__running.set()	  # 将running设置为True

	def run(self):
		while self.__running.isSet():
			self.__flag.wait()	  # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
			with canvas(device) as draw:
				draw.text((0, 0), text_1, fill="white")
				draw.text((0, 10), text_2, fill="white")
				draw.text((0, 20), text_3, fill="white")
				draw.text((0, 30), text_4, fill="white")
				draw.text((0, 40), text_5, fill="white")
				draw.text((0, 50), text_6, fill="white")
			print('loop')
			self.pause()

	def pause(self):
		self.__flag.clear()	 # 设置为False, 让线程阻塞

	def resume(self):
		self.__flag.set()	# 设置为True, 让线程停止阻塞

	def stop(self):
		self.__flag.set()	   # 将线程从暂停状态恢复, 如何已经暂停的话
		self.__running.clear()		# 设置为False  

	def screen_show(self, position, text):
		global text_1, text_2, text_3, text_4, text_5, text_6
		if position == 1:
			text_1 = text
		elif position == 2:
			text_2 = text
		elif position == 3:
			text_3 = text
		elif position == 4:
			text_4 = text
		elif position == 5:
			text_5 = text
		elif position == 6:
			text_6 = text
		self.resume()

if __name__ == '__main__':
	screen = OLED_ctrl()
	screen.start()
	while 1:
		time.sleep(10)
		pass
```

# Session 1: Ball Tracking Robot 
A robot that is able to use computer vision to track a ball. After tracking the ball, the robot will move towards the ball. 

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ahanaf Z | The Browning School | Astronautical Engineering | Incoming Junior

# Final Robot
![finalrobot](https://user-images.githubusercontent.com/88998860/176932135-a32dfffe-0276-4549-a8d0-11d69060b3d0.jpg)


# Schematic
![schematic](https://user-images.githubusercontent.com/88998860/176747289-a454e52e-2899-4ae8-b3b3-27a2c526a270.jpg)

# Demo Night 
<iframe width="800" height="450" src="https://www.youtube.com/embed/fQA_HxivaQc" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Third Milestone
My third milestone marks the end of the base robot build for my project and opens up a lot of new doors for modifications to make my robot much cooler than it already is. The robot hasn't changed drastically from the second milestone but does have a key difference from it's previous model and that is the object tracking part. More specifically speaking, the robot is now able to use bounding rectangle calculations, contours, and aid from image masking to detect objects through the pi-camera and move towards that object through a number of conditionals and PWM operations on the motors. I look forward to slowly turning my simple "ball-tracking-robot" into a more rover like robot in the next 3 weeks of Bluestamp I plan on attending and am really excited about it!  

<iframe width="800" height="450" src="https://www.youtube.com/embed/aqm2EQHTo_w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Second Milestone
My second milestone was a lot more hands-on relatively speaking than my first milestone as I got to work with a lot of the hardware for my project such as the raspberrypi pins, jumper cables, a bread board, ultrasonic sensors, encoders, h-bridges and many others. I quickly learned how the different pins on a raspberry pi function and got right to building the chassi for my robot as well as attaching motors, wheels, and finding a reliable power source for the robot. I used tape as my main adhesive for keeping most of my hardware on the robot chassi itself. I then proceeded to write code to move my motors using PWM as well as code for evaluating distance through the ultra sonic sensors. I proceeded to use a breadboard to make the use of the ultrasonic sensors more organized if nothing else as all the cables on my raspberry pi were getting very messy. Finally, after a day of troubleshooting, I realized that the double A batteries I was originally using for my h-bridge that had originally been working, could notpower the h-bridge, raspberry pi, breadboard, ultrasonic sensors, and motors all at the same time. Hence, I replaced my primary battery source with a phone battery bank, and for the time being moved my code from VNC Viewer to Visual Studio Code where I was able to view it more easily. With all of this working, I coded the logic for my obstacle avoiding robot (the 2nd step in getting the full ball tracking robot working, as I didn't want my robot to run into walls.) One big obstacle was the battery and uneven voltages, but luckily the battery bank solved this issue. 

<iframe width="800" height="450" src="https://www.youtube.com/embed/3l1YpAXBgGA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# First Milestone
  

My first milestone included mainly included working on the software side of things for my ball tracking robot. First I began with downloading all the required libraries and dependences like PyQT, OpenCV, PiCamera2, Numpy, and a few others. Afterwards, I used the PiCamera2 documentation along with help from my instructor to construct an image masking/color filtering program with my PiCamera through VNC Viewer. The image masker works by collecting BGR values from a PiCamera frame (or RGB values if specified in the configuration parameters), and then uses OpenCV (imported as CV2 in the code) to translate the BGR/RGB values into HSV (Hue saturation values). This is so that when the program uses the picamera to search for color, it can search for a range of colors instead of one very specific RGB or BGR value. 


<iframe width="800" height="450" src="https://www.youtube.com/embed/018_zv7ilfU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Session 1 Code

```py
import RPi.GPIO as GPIO
import time
from time import sleep
import cv2
import numpy as np
from picamera2 import Picamera2, Preview

def measure_distance(sensor_dict):
    GPIO.setup(sensor_dict['TRIG'], GPIO.OUT)
    GPIO.output(sensor_dict['TRIG'], 0)
    GPIO.setup(sensor_dict['ECHO'], GPIO.IN)

    time.sleep(0.1)

    GPIO.output(sensor_dict['TRIG'], 1)
    time.sleep(0.00001)
    GPIO.output(sensor_dict['TRIG'], 0)

    while GPIO.input(sensor_dict['ECHO']) == 0:
        pass
    start = time.monotonic()
    while GPIO.input(sensor_dict['ECHO']) == 1:
        pass
    stop = time.monotonic()

    distance = round((((stop - start) * 17000)), 1)
    '''print(f"{distance}", end="   ")'''

    return distance

def find_blob(blob):
    largest_contour = 0
    cont_index = 0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour = area

            cont_index = idx
    
    r = (0, 0, 2, 2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r, largest_contour    

def stop():
    m1_pwm.stop()
    m2_pwm.stop()
    m3_pwm.stop()
    m4_pwm.stop()

def forwards(speed):
    m1_pwm.start(speed)
    m2_pwm.start(0)
    m3_pwm.start(speed)
    m4_pwm.start(0)

def backwards(speed):
    m1_pwm.start(0)
    m2_pwm.start(speed)
    m3_pwm.start(0)
    m4_pwm.start(speed)

def turn_left(speed):
    m1_pwm.start(speed)
    m2_pwm.start(0)
    m3_pwm.start(0)
    m4_pwm.start(speed)

def turn_right(speed):
    m1_pwm.start(0)
    m2_pwm.start(speed)
    m3_pwm.start(speed)
    m4_pwm.start(0)


GPIO.setmode(GPIO.BCM)

# Left motor (Weaker)
mi_1 = 13
mi_2 = 19

# Right Motor 
mi_3 = 12
mi_4 = 18

GPIO.setup(mi_1,GPIO.OUT)
GPIO.setup(mi_2,GPIO.OUT)
GPIO.setup(mi_3,GPIO.OUT)
GPIO.setup(mi_4,GPIO.OUT)

m1_pwm = GPIO.PWM(mi_1, 35)
m2_pwm = GPIO.PWM(mi_2, 35)
m3_pwm = GPIO.PWM(mi_3, 35)
m4_pwm = GPIO.PWM(mi_4, 35)


# Sensor data setup 
sens1 = {
    'TRIG' : 23,
    'ECHO' : 14,
    'NAME' : 'SensorFront'
}
sens2 = {
    'TRIG' : 15,
    'ECHO' : 24,
    'NAME' : 'SensorLeft'
}
sens3 = {
    'TRIG': 17,
    'ECHO': 27,
    'NAME': 'SensorRight'
}


cv2.startWindowThread()

picam2 = Picamera2()

config = picam2.preview_configuration(main={"size": (256, 144), "format": "RGB888"})

picam2.configure(config)

picam2.start()



while True:
    try:
        print('---------------')
        im = picam2.capture_array()
        centre_x = 0.
        centre_y = 0.

        hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
        lower_range = np.array([120,150,50])
        upper_range = np.array([255,255,255])
        
        mask = cv2.inRange(hsv, lower_range, upper_range)
        loct, area = find_blob(mask)

        
        x, y, w, h = loct
        print('width:' + str(w) + ' height:' + str(h))

        # Put in code for getting distance from all 3 sensors 

        s1_d = measure_distance(sens1)
        s2_d = measure_distance(sens2)
        s3_d = measure_distance(sens3)

        if (w*h) < 600:
            found=0
        else:
            found=1
            simg2 = cv2.rectangle(im, (x,y), (x+w,y+h), 255,2)
            centre_x=x+((w)/2)
            centre_y=y+((h)/2)
            cv2.circle(im,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
            centre_x-=80
            centre_y=6--centre_y
            print('Center x: ' + str(centre_x) + ' --- Center y: ' + str(centre_y))
        initial=600
        flag=0
        
        cv2.imshow('draw', im)
        cv2.imshow('mask', mask)

        if(found==0):
            print('Found 0 activated')

        #if the ball is not found and the last time it sees ball in which direction, it will start to rotate in that direction
            if flag==0:
                print('Turning left.')
                turn_right(50)
                time.sleep(0.3)
            else:
                print('Turning right')
                turn_left(50)
                time.sleep(0.3)
            stop()
            time.sleep(0.0125)
        
        elif(found==1):
            print('Found 1 activated')
            print('area: ' + str(area))
            if(area>=initial):
                initial2=18000
                if(area<initial2):
                    print('s1_d is: ' + str(s1_d))
                    if(s1_d>10):
                        print('sensor 1 distance is greater than 10')
                        if(centre_x<=-20 or centre_x>=20):
                            if(centre_x<0):
                                flag=0
                                turn_right(50)
                                time.sleep(0.2)
                            elif(centre_x>0):
                                flag=1
                                turn_left(50)
                                time.sleep(0.2)
                        forwards(60)
                        time.sleep(0.6)
                        stop()
                        time.sleep(0.1)
                        
                    else:
                        print('stopping at the else')
                        stop()
                        time.sleep(0.01)
            
                    
    except KeyboardInterrupt:
        print('Program closing.')
        break

GPIO.cleanup()

```

| **Quantity & Part Name** | **Part Description** | **Reference Designators** | **Cost** | **Link** |
|:--:|:--:|:--:|:--:|:--|
| 1 Screw Driver Kit | Included different screws & screw driver | N/A | $5.94 | https://www.amazon.com/Small-Screwdriver-Set-Mini-Magnetic/dp/B08RYXKJW9/ | 
| 3 Ping Sensors | Used for distance readings | HC - SR04 | $12.99 | https://www.amazon.com/ELEGOO-HC-SR04-Ultrasonic-Distance-MEGA2560/dp/B01COSN7O6/ |
| 1 H-Bridge | Used to power robot motors | X | $8.99 | https://www.amazon.com/ACEIRMC-Stepper-Controller-2-5-12V-H-Bridge/dp/B0923VMKSZ/ | 
| 1 Pi-Cam | Used for object detection and filtering | N/A | $9.99 | https://www.amazon.com/Arducam-Megapixels-Sensor-OV5647-Raspberry/dp/B012V1HEP4/ |
| 1 Set Jumpter Wires | Used for transferring voltage from object to object | N/A | $6.98 | https://www.amazon.com/Elegoo-EL-CP-004-Multicolored-Breadboard-arduino/dp/B01EV70C78/ |
| 1 Sodlering Kit | Used to melt metal together with solder | N/A | $19.99 | https://www.amazon.com/Soldering-Iron-Kit-Temperature-Screwdrivers/dp/B07GJNKQ8W/ref=sr_1_4_sspa?crid=3SWW7HN9U1AF1&keywords=20+dollar+soldering+kit&qid=1656613484&sprefix=20+dollar+soldering+kit%2Caps%2C59&sr=8-4-spons&psc=1&smid=A2CEQAD2VNOS6B&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFBSkI4UURNT0tRSlomZW5jcnlwdGVkSWQ9QTAwMzg5ODIzVkk0Nk03V1pSTzFRJmVuY3J5cHRlZEFkSWQ9QTA5Nzk4MTIxTUNMRUUwMlpWMlk1JndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ== | 
| 4 PC Breadboard Kit | Used for sensor wire organization | N/A | $11.98 | https://www.amazon.com/Breadboards-Solderless-Breadboard-Distribution-Connecting/dp/B07DL13RZH/ref=sr_1_1_sspa?crid=1TVQWGWIN5I4Q&keywords=breadboard&qid=1656613593&s=industrial&sprefix=breadboard%2Cindustrial%2C72&sr=1-1-spons&psc=1&smid=AX8SR0V05IQ2E&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyNzQ1UFlYMTZLS0oxJmVuY3J5cHRlZElkPUEwNDI3MzA3MzdRTEk0MU43WTg0RSZlbmNyeXB0ZWRBZElkPUEwNzgyNTA2Mzk3RkFTTTg1Qk9INSZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU= |
| 1 Power Bank | Used to power raspberry pi | N/A | $9.65 | https://www.amazon.com/Emilykylie-Multicolor-Cylinder-Battery-Charging/dp/B08YN9TL6P/ref=sr_1_3?crid=IWWD3H3E5Y5L&keywords=battery%2Bbank%2Bcylinder&qid=1656613614&s=electronics&sprefix=battery%2Bbank%2Bcylinder%2Celectronics%2C52&sr=1-3&th=1 |
| 1 Raspberry Pi 4 | Main hub of robot for functionality | N/A | $124.99 | https://www.amazon.com/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27/ |
| 1 Micro SD Card | Used to transfer Pi data | N/A | $8.00 | https://www.amazon.com/gp/product/B089VVP61W/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1 | 
| 1 Portable Card Reader | Used to read Micro SD Card | N/A | $12.99 | https://www.amazon.com/dp/B07ZKRM12C/ref=sspa_dk_detail_0?psc=1&pd_rd_i=B07ZKRM12C&pd_rd_w=PMv06&pf_rd_p=7771f1a2-d77a-4098-a19e-6d9a1e65f44d&pd_rd_wg=WZWTX&pf_rd_r=N0S0M3D2VYYXTFJS0CYA&pd_rd_r=cb58c4a4-5003-4566-a6c9-b82133f4f19d&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFWMUUwWElYR045V0UmZW5jcnlwdGVkSWQ9QTA2NzgwNTMxQUNMVFA5NUg5SUxJJmVuY3J5cHRlZEFkSWQ9QTA0NDU5OTczM1JBTVg1M1dINFhWJndpZGdldE5hbWU9c3BfZGV0YWlsJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ== |
| Raspberry Pi Power Supply | Used to power raspberry pi wired | N/A | $7.99 | https://www.amazon.com/Replacement-Raspberry-Pi-4-Supply-Charger-Adapter/dp/B094J8TK61/ |

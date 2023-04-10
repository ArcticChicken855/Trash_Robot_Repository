import RPi.GPIO as GPIO
import time
import numpy as np
import cv2

# Define GPIO pins for motor control
left_forward_pin = 17
left_backward_pin = 18
right_forward_pin = 27
right_backward_pin = 22

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_forward_pin, GPIO.OUT)
GPIO.setup(left_backward_pin, GPIO.OUT)
GPIO.setup(right_forward_pin, GPIO.OUT)
GPIO.setup(right_backward_pin, GPIO.OUT)

# Define functions for motor control
def forward():
    GPIO.output(left_forward_pin, GPIO.HIGH)
    GPIO.output(right_forward_pin, GPIO.HIGH)

def backward():
    GPIO.output(left_backward_pin, GPIO.HIGH)
    GPIO.output(right_backward_pin, GPIO.HIGH)

def left():
    GPIO.output(left_backward_pin, GPIO.HIGH)
    GPIO.output(right_forward_pin, GPIO.HIGH)

def right():
    GPIO.output(left_forward_pin, GPIO.HIGH)
    GPIO.output(right_backward_pin, GPIO.HIGH)

def stop():
    GPIO.output(left_forward_pin, GPIO.LOW)
    GPIO.output(left_backward_pin, GPIO.LOW)
    GPIO.output(right_forward_pin, GPIO.LOW)
    GPIO.output(right_backward_pin, GPIO.LOW)

# Define GPIO pins for ultrasonic sensors
trig_pins = [23, 24, 25]
echo_pins = [16, 20, 21]

# Set up GPIO pins for ultrasonic sensors
for i in range(len(trig_pins)):
    GPIO.setup(trig_pins[i], GPIO.OUT)
    GPIO.setup(echo_pins[i], GPIO.IN)

# Define function for ultrasonic sensor measurement
def measure_distance(trig_pin, echo_pin):
    # Send ultrasonic pulse
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)
    
    # Measure pulse duration
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
    
    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

# Define function for robot movement based on obstacle detection
def move():
    distances = [measure_distance(trig_pins[i], echo_pins[i]) for i in range(len(trig_pins))]
    min_distance = min(distances)
    
    if min_distance < 20: # obstacle detected
        stop()
        time.sleep(0.5)
        left_or_right = np.random.choice([left, right])
        left_or_right()
        time.sleep(0.5)
    else:
        forward()

# Define OpenCV object detection function
def detect_objects():
    # Define color ranges for object detection
    blue_lower = np.array([100, 50, 50])
    blue_upper = np.array([130, 255, 255])
    red_lower = np.array([0, 50, 50])
    red_upper = np.array([10, 255, 255])
    green_lower = np.array([50, 50, 50])
    green_upper = np.array([70, 255

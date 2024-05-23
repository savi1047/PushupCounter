import cv2
import mediapipe as mp
import os
import time
import RPi.GPIO as GPIO
import pygame
from gpiozero import MotionSensor


pygame.mixer.init()

# Initialize MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Keypad GPIO pins
L1, L2, L3, L4 = 25, 8, 7, 1
C1, C2, C3, C4 = 12, 16, 20, 21
light_pin = 18  # GPIO pin connected to the light
pir = MotionSensor(4)
# Setup GPIO for LED and keypad
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(light_pin, GPIO.OUT)  # Set pin as output
GPIO.setup([L1, L2, L3, L4], GPIO.OUT)
GPIO.setup([C1, C2, C3, C4], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# Variables for push-up detection
counter = 0
stage = None
create = None
opname = "output.avi"
pushup_count_threshold = 5

# Video capture
cap = cv2.VideoCapture(0)

# Saved password and keypad settings
saved_password = ['1', '2', '3', '4']
input_code = []
setting_mode = False
password_mode = False
debounce_time = 0.3  # Debouncing time for keypad input

def play_sound():
    file_name ="sample1.wav"
    full_path= ("/home/savi/sample1.wav")
    pygame.mixer.music.load(full_path)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

pir.when_motion = play_sound

def findPosition(image, draw=True):
    lmList = []
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        for id, lm in enumerate(results.pose_landmarks.landmark):
            h, w, c = image.shape
            cx, cy = int(lm.x * w), int(lm.y * h)
            lmList.append([id, cx, cy])
    return lmList

def readLine(line, characters):
    GPIO.output(line, GPIO.HIGH)
    key_pressed = None
    if GPIO.input(C1) == 1:
        key_pressed = characters[0]
    elif GPIO.input(C2) == 1:
        key_pressed = characters[1]
    elif GPIO.input(C3) == 1:
        key_pressed = characters[2]
    elif GPIO.input(C4) == 1:
        key_pressed = characters[3]
    GPIO.output(line, GPIO.LOW)
    return key_pressed

with mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7) as pose:
    while cap.isOpened():
        success, image = cap.read()
        image = cv2.resize(image, (640, 480))

        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        lmList = findPosition(image, draw=True)

        # Handle push-up detection
        if len(lmList) != 0:
            shoulder1 = lmList[11][2]  # Right shoulder Y coordinate
            shoulder2 = lmList[12][2]  # Left shoulder Y coordinate
            elbow1 = lmList[13][2]  # Right elbow Y coordinate
            elbow2 = lmList[14][2]  # Left elbow Y coordinate

            if shoulder1 < elbow1 and shoulder2 < elbow2:
                if stage == "down":
                    stage = "up"
                    counter += 1
                    print(f"Push-up Count: {counter}")
                    if counter >= pushup_count_threshold:
                        GPIO.output(light_pin, GPIO.HIGH)
                        time.sleep(10)
                        GPIO.output(light_pin, GPIO.LOW)
                        counter = 0  # Reset counter after reaching threshold
                        print("Light turned on and then off after 10 seconds")
            else:
                stage = "down"

        # Handle keypad input
        key = None
        for (line, chars) in zip([L1, L2, L3, L4], [["1", "2", "3", "A"], ["4", "5", "6", "B"], ["7", "8", "9", "C"], ["*", "0", "#", "D"]]):
            key = readLine(line, chars)
            if key:
                print(f"Key pressed: {key}")
                break  # Exit after the first key is detected
        
        if key:
            time.sleep(debounce_time)  # Debounce delay
            if key == '*' and not setting_mode and not password_mode:
                setting_mode = True
                input_code = []
                print("Setting mode: Enter new 4-digit password")
            elif key == '#' and setting_mode and len(input_code) == 4:
                saved_password = input_code.copy()
                setting_mode = False
                print("New password saved")
            elif key == 'A' and not setting_mode:
                password_mode = True
                input_code = []
                print("Enter your password")
            elif setting_mode or password_mode:
                input_code.append(key)
                if len(input_code) == 4 and password_mode:
                    if input_code == saved_password:
                        print("Open")
                        GPIO.output(light_pin, GPIO.HIGH)  # Turn on the light
                        time.sleep(5)  # Light stays on for 5 seconds
                        GPIO.output(light_pin, GPIO.LOW)  # Turn off the light
                    else:
                        print("Close")
                    password_mode = False  # Exit password mode after attempt

        text = f"Push Ups: {counter}"
        cv2.putText(image, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow('MediaPipe Pose', image)
        if create is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            create = cv2.VideoWriter(opname, fourcc, 30, (image.shape[1], image.shape[0]), True)
        create.write(image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
GPIO.cleanup()



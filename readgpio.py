import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False) 

DIR = 15
STEP=13
RESET = 40

CW=1
CCW=0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(STEP,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(RESET,GPIO.IN)
while True:
    preStatus = GPIO.input(RESET)
    print(preStatus)




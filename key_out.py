#!/usr/bin/python3.7

#sudo rtl_433 -f 433890000 -F json | /usr/bin/python3.7 key_out.py

import json
import RPi.GPIO as GPIO
import sys
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(32,GPIO.OUT)
GPIO.setup(29,GPIO.OUT)

def get_rtl_433_data():
   for line in sys.stdin:
      try:
         data = json.loads(line)

         print(data["code"])

         if data["code"] in ['85e772']:
            print("Button A")
            print ("Red LED on")
            GPIO.output(18,GPIO.HIGH)
            time.sleep(2)
            print ("Red LED off")
            GPIO.output(18,GPIO.LOW)

         if data["code"] in ['45e772']:
            print("Button B")
            print ("Blue LED on")
            GPIO.output(22,GPIO.HIGH)
            time.sleep(2)
            print ("Blue LED off")
            GPIO.output(22,GPIO.LOW)

         if data["code"] in ['25e772']:
            print("Button C")
            print ("Green LED on")
            GPIO.output(32,GPIO.HIGH)
            time.sleep(2)
            print ("Green LED off")
            GPIO.output(32,GPIO.LOW)

         if data["code"] in ['15e772']:
            print("Button D")
            print ("Yellow LED on")
            GPIO.output(29,GPIO.HIGH)
            time.sleep(2)
            print ("Yellow LED off")
            GPIO.output(29,GPIO.LOW)

      except KeyError:
            print("KeyError exception")
            break
      except ValueError:
            print("ValueError exception")
            break

def main():
   get_rtl_433_data()
   
if __name__ == "__main__":
   main()

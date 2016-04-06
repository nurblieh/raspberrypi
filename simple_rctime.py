#!/usr/bin/env python
 
# Simple example of RC timing for the Raspberry Pi.
#
# Usage: simple_rctime.py <pin#>
#
# Output: Time (in sec) for the voltage across the capacitor to reach
# GPIO.HIGH, which according to the BCM documentation is 1.3V and in
# my experience is closer to 1.44V.
# With this you can determine the resistence of your resistor.

import sys
import time

import RPi.GPIO as GPIO

DEBUG = 1
GPIO.setmode(GPIO.BCM)
 

def RCTime(pin):
  # Reset the pin we'll be using to perform our reading.
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, GPIO.LOW)
  time.sleep(0.1)
 
  GPIO.setup(pin, GPIO.IN)
  t = time.time()
  # wait_for_edge() blocks until the pin goes from LOW to HIGH.
  GPIO.wait_for_edge(pin, GPIO.RISING)
  return time.time() - t


def main(pin):
  try:
    while True:
        print RCTime(pin)

  finally:
    GPIO.cleanup()


if __name__ == '__main__':
  main(int(sys.argv[1]))

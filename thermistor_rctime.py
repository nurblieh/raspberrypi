#!/usr/bin/env python
 
# Print temperature given a thermistor+capacitor RC circuit.
# For use with the Raspberry Pi and the RPi.GPIO library.
#
# The Thermistor I used to test this is available from Adafruit.
# https://www.adafruit.com/products/372
# Here is the datasheet they provide.
# https://www.adafruit.com/datasheets/103_3950_lookuptable.pdf
#
# The other important bit of calibration is: at what voltage does
# your Raspberry Pi pins read HIGH. I've seen docs that say >=1.3V
# and also >=1.8V. In my experience (using a few 10K resistors as calibration)
# my pins trip HIGH at 1.44V. You should calibrate similarly.
#
# Usage: thermistor_rctime.py --help
#
# Author: Bradley Heilbrun <nurblieh@gmail.com>

import argparse
import math
import sys
import time

import RPi.GPIO as GPIO

DEBUG = 1
GPIO.setmode(GPIO.BCM)

class RCCircuit(object):
  '''RCCircuit class.

  Simple class to convert time (in sec) to resistence (in ohms), given a
  calibrated RC series circuit. More exactly, if we know the capacitance
  of C (in ohms) and the time for C to reach Vc (in volts) supplied by source V
  (in volts) we can determine the resistence of R (in ohms).

  The wikipedia article isn't so helpful, but might be a useful reference,
  https://en.wikipedia.org/wiki/RC_circuit
  '''
  def __init__(self, C, Vc, V):
    self.C = C
    self.Vc = Vc
    self.V = V

  def TimeToResistence(self, t):
    '''RC (series) circuit equation solved for R (ohms).
    https://en.wikipedia.org/wiki/RC_circuit#Time-domain_considerations

    Args:
      t (float): Time spent for Capacitor C to reach voltage Vc.
    Returns:
      (float): Resistence of in-line resistor sensor (eg, thermistor).
    '''
    return t / (math.log(1 / (1 - self.Vc / self.V)) * self.C)


class Thermistor(object):
  '''Thermistor class.

  Using the Steinhart-Hart equation we can accurately estimate the temp
  if we know certain constants about our thermistor. These should be
  provided by the thermistor manufacturer.
  https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
  '''
  def __init__(self, B, RZero, TZero):
    '''
    Args:
      B (int): Thermistor coefficient.
      RZero (int): Resistence (in ohms) at point 0.
      TZero (float): Temp (in kelvins) at point 0.
    '''
    # Coerce all variables to float.
    self.B = float(B)
    self.RZero = float(RZero)
    self.TZero = float(TZero)

  def ResistenceToTemp(self, r):
    '''Convert resistence (ohms) to temperature.

    Args:
      r (float): Resistence in ohms of the thermistor.
    Returns:
      (float): Temperature in Kelvin.
    '''
    return 1.0 / (
      (1.0 / self.TZero) + (1.0 / self.B) * math.log(r / self.RZero)
      )


def KelvinToCelsius(k):
  '''Convert temperature into something common.

  Args:
    k (float): Temperature in Kelvin
  Returns:
    (float): Temperature in Celsius
  '''
  return k - 273.15


def RCTime(pin):
  # Reset the pin we'll be using to perform our reading.
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, GPIO.LOW)
  time.sleep(0.1)
 
  GPIO.setup(pin, GPIO.IN)
  t = time.time()
  # wait_for_edge() blocks until the pin goes from LOW to HIGH (in this case).
  GPIO.wait_for_edge(pin, GPIO.RISING)
  return time.time() - t


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--pin', type=int, default=18,
                      help='Pin to read on the Pi. BCM-reckoning.')
  parser.add_argument('-B', '--thermistor_coefficient',
                      type=float, default=3950.0,
                      help='B coefficient of the thermistor. Provided '
                          'by the manufacturer.')
  parser.add_argument('--RZero', type=float, default=10000,
                      help='Resistence at point 0. Provided by manufacturer.')
  parser.add_argument('--TZero', type=float, default=298.15,
                      help='Temperature to match RZero. Provided by mfg.')
  parser.add_argument('-C', '--capacitance',
                      type=float, default=100*1e-6,
                      help='Capacitance of your capacitor in farads.')
  parser.add_argument('--Vc', '--capacitor_voltage',
                      type=float, default=1.44,
                      help='The voltage at which your pin reads HIGH. '
                          'Broadcom documentation says 1.3V, but my '
                          'experience says 1.44V.')
  parser.add_argument('-V', '--supply_voltage',
                      type=float, default=3.3,
                      help='Voltage provided by power source. You should only '
                          'use 3.3V with logic pins.')
  args = parser.parse_args()

  circuit = RCCircuit(args.capacitance,
                      args.Vc,
                      args.supply_voltage)
  thermistor = Thermistor(args.thermistor_coefficient,
                          args.RZero,
                          args.TZero)
  try:
    while True:
        time = RCTime(args.pin)
        r = circuit.TimeToResistence(time)
        temp = thermistor.ResistenceToTemp(r)
        print [time, r, KelvinToCelsius(temp)]
        
  finally:
    # We need to release the pins when we're done!
    GPIO.cleanup()


if __name__ == '__main__':
  main()

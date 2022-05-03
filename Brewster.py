

CH_B_GAIN_32  = 2 # Channel B gain 32
CH_A_GAIN_64  = 0 # Channel A gain 64
CH_A_GAIN_128 = 1 # Channel A gain 128


DATA_CLKS = 24
X_128_CLK = 25
X_32_CLK  = 26
X_64_CLK  = 27

PULSE_LEN = 15

# If the data line goes low after being high for at least
# this long it indicates that a new reading is available.

TIMEOUT = ((X_64_CLK + 3) * 2 * PULSE_LEN)

# The number of readings to skip after a mode change.

SETTLE_READINGS = 5

from multiprocessing.connection import wait
import time
from tkinter import W

import pigpio # http://abyz.co.uk/rpi/pigpio/python.html


L = [0,2993.732, 6113.855, 9258.483, 12341.969, 15618.807, 18702.044, 21879.511, 
      24902.654, 27947.354, 31120.356, 34418.718, 37517.239, 40546.607, 
      43858.074, 47081.914, 49907.934]
R = [0,3261.040, 6485.505, 9408.287, 12633.759, 15780.681, 18968.484, 22033.977, 
      25132.522, 28115.462, 31351.280, 34585.654, 37697.145, 40716.273, 
      44067.366, 47264.058, 50137.398]

D1L=[L[2],L[4],L[5]]
D1R=[R[2],R[4],R[5]]
D2L=[L[2],L[4],L[5]]
D2R=[R[2],R[4],R[5]]
D3L=[L[2],L[6],L[8]]
D3R=[R[2],R[6],R[8]]
D4L=[4559.326,16166.199]
D4R=[4826.634,16583.3119]
D5L=[L[3],L[4],17184.401]
D5R=[R[3],R[4],17346.27]
D6L=[4559.326]
D6R=[4826.634]
         
class sensor:

   """
   A class to read the HX711 24-bit ADC.
   """

   def __init__(self, pi, DATA, CLOCK, mode=CH_A_GAIN_128, callback=None):
      """
      Instantiate with the Pi, the data GPIO, and the clock GPIO.

      Optionally the channel and gain may be specified with the
      mode parameter as follows.

      CH_A_GAIN_64  - Channel A gain 64
      CH_A_GAIN_128 - Channel A gain 128
      CH_B_GAIN_32  - Channel B gain 32

      Optionally a callback to be called for each new reading may be
      specified.  The callback receives three parameters, the count,
      the mode, and the reading.  The count is incremented for each
      new reading.
      """
      self.pi = pi
      self.DATA = DATA
      self.CLOCK = CLOCK
      self.callback = callback

      self._paused = True
      self._data_level = 0
      self._clocks = 0

      self._mode = CH_A_GAIN_128
      self._value = 0

      self._rmode = CH_A_GAIN_128
      self._rval = 0
      self._count = 0

      self._sent = 0
      self._data_tick = pi.get_current_tick()
      self._previous_edge_long = False
      self._in_wave = False

      self._skip_readings = SETTLE_READINGS

      pi.write(CLOCK, 1) # Reset the sensor.

      pi.set_mode(DATA, pigpio.INPUT)

      pi.wave_add_generic(
         [pigpio.pulse(1<<CLOCK, 0, PULSE_LEN),
          pigpio.pulse(0, 1<<CLOCK, PULSE_LEN)])

      self._wid = pi.wave_create()

      self._cb1 = pi.callback(DATA, pigpio.EITHER_EDGE, self._callback)
      self._cb2 = pi.callback(CLOCK, pigpio.FALLING_EDGE, self._callback)

      self.set_mode(mode)

   def get_reading(self):
      """
      Returns the current count, mode, and reading.

      The count is incremented for each new reading.
      """
      return self._count, self._rmode, self._rval

   def set_callback(self, callback):
      """
      Sets the callback to be called for every new reading.
      The callback receives three parameters, the count,
      the mode, and the reading.  The count is incremented
      for each new reading.

      The callback can be cancelled by passing None.
      """
      self.callback = callback

   def set_mode(self, mode):
      """
      Sets the mode.

      CH_A_GAIN_64  - Channel A gain 64
      CH_A_GAIN_128 - Channel A gain 128
      CH_B_GAIN_32  - Channel B gain 32
      """
      self._mode = mode

      if mode == CH_A_GAIN_128:
         self._pulses = X_128_CLK
      elif mode == CH_B_GAIN_32:
         self._pulses = X_32_CLK
      elif mode == CH_A_GAIN_64:
         self._pulses = X_64_CLK
      else:
         raise ValueError

      self.pause()
      self.start()

   def pause(self):
      """
      Pauses readings.
      """
      self._skip_readings = SETTLE_READINGS
      self._paused = True
      self.pi.write(self.CLOCK, 1)
      time.sleep(0.002)
      self._clocks = DATA_CLKS + 1

   def start(self):
      """
      Starts readings.
      """
      self._wave_sent = False
      self.pi.write(self.CLOCK, 0)
      self._value = 0
      self._paused = False
      self._skip_readings = SETTLE_READINGS

   def cancel(self):
      """
      Cancels the sensor and release resources.
      """
      self.pause()

      if self._cb1 is not None:
         self._cb1.cancel()
         self._cb1 = None

      if self._cb2 is not None:
         self._cb2.cancel()
         self._cb2 = None

      if self._wid is not None:
         self.pi.wave_delete(self._wid)
         self._wid = None

   def _callback(self, gpio, level, tick):

      if gpio == self.CLOCK:

         if level == 0:

            self._clocks += 1

            if self._clocks <= DATA_CLKS:

               self._value = (self._value << 1) + self._data_level

               if self._clocks == DATA_CLKS:

                  self._in_wave = False

                  if self._value & 0x800000: # unsigned to signed
                     self._value |= ~0xffffff

                  if not self._paused:

                     if self._skip_readings <= 0:

                        self._count = self._sent
                        self._rmode = self._mode
                        self._rval = self._value + 86849.33

                        if self.callback is not None:
                           self.callback(self._count, self._rmode, self._rval)

                     else:
                        self._skip_readings -= 1

      else:

         self._data_level = level

         if not self._paused:

            if self._data_tick is not None:

               current_edge_long = pigpio.tickDiff(
                  self._data_tick, tick) > TIMEOUT

            if current_edge_long and not self._previous_edge_long:

               if not self._in_wave:

                  self._in_wave = True

                  self.pi.wave_chain(
                     [255, 0, self._wid, 255, 1, self._pulses, 0])

                  self._clocks = 0
                  self._value = 0
                  self._sent += 1

         self._data_tick = tick
         #self._previous_edge_long = current_edge_long


if __name__ == "__main__":
    import time
    import pigpio
    import HX711


    import os
    open_io="sudo pigpiod"
    os.system(open_io)
    time.sleep(1)

    def cbf(count, mode, reading):
      print(count, mode, reading)

    pi = pigpio.pi()

    rec=[0,2,3,4,17,27,22]
    send=[0,10,9,11,26,19,13]
    pump=[0,14,15,18,23,24,25]
    vib=8

    
    pi.set_mode(pump[1], pigpio.OUTPUT)
    pi.set_mode(pump[2], pigpio.OUTPUT)
    pi.set_mode(pump[3], pigpio.OUTPUT)
    pi.set_mode(pump[4], pigpio.OUTPUT)
    pi.set_mode(pump[5], pigpio.OUTPUT)
    pi.set_mode(pump[6], pigpio.OUTPUT)
    pi.set_mode(vib, pigpio.OUTPUT)

    
    pi.set_pull_up_down(rec[1], pigpio.PUD_DOWN)
    pi.set_pull_up_down(rec[2], pigpio.PUD_DOWN)
    pi.set_pull_up_down(rec[3], pigpio.PUD_DOWN)
    pi.set_pull_up_down(rec[4], pigpio.PUD_DOWN)
    pi.set_pull_up_down(rec[5], pigpio.PUD_DOWN)
    pi.set_pull_up_down(rec[6], pigpio.PUD_DOWN)

    pi.set_mode(send[1], pigpio.OUTPUT)
    pi.set_mode(send[2], pigpio.OUTPUT)
    pi.set_mode(send[3], pigpio.OUTPUT)
    pi.set_mode(send[4], pigpio.OUTPUT)
    pi.set_mode(send[5], pigpio.OUTPUT)
    pi.set_mode(send[6], pigpio.OUTPUT)

    s = HX711.sensor(pi, DATA=20, CLOCK=21, mode=HX711.CH_B_GAIN_32, callback=cbf)
    
    s.set_callback(None)
    s.set_mode(HX711.CH_A_GAIN_128)   

    c, mode, reading = s.get_reading()
    stop = time.time() + 3600
    t=0
    w=0
    n=0
    x=0
    H=0
    stopcount=0
    pi.write(pump[1], 0)
    pi.write(pump[2], 0)
    pi.write(pump[3], 0)
    pi.write(pump[4], 0)
    pi.write(pump[5], 0)
    pi.write(pump[6], 0)
    pi.write(vib,0)
    pi.write(send[1],0)
    pi.write(send[2],0)
    pi.write(send[3],0)
    pi.write(send[4],0)
    pi.write(send[5],0)
    pi.write(send[6],0)
    while time.time() < stop:
  
        

        while H==0:
            #x = int(input("Select your drink:"))
            #print ("You have selected", x)
            #H=1
            if pi.read(rec[1])==1:
                 x=1
                 print ("You have selected", x)
                 H=1
            elif pi.read(rec[2])==1:
                 x=2
                 print ("You have selected", x)
                 H=1
            elif pi.read(rec[3])==1:
                 x=3
                 print ("You have selected", x)
                 H=1
            elif pi.read(rec[4])==1:
                 x=4
                 print ("You have selected", x)
                 H=1
            elif pi.read(rec[5])==1:
                 x=5
                 print ("You have selected", x)
                 H=1
            elif pi.read(rec[6])==1:
                 x=6
                 print ("You have selected", x)
                 H=1
        count, mode, reading = s.get_reading() 
        if count != c: 
                
            if x==1:         #drink 1 selected 
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[1],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D1L[0] < weight < D1R[0]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(pump[5],1)
                    if D1L[1] < weight < D1R[1]:          
                        pi.write(pump[5],0)
                        time.sleep(0.001)
                        pi.write(pump[6],1)
                    if D1L[2] < weight < D1R[2]:         
                        pi.write(pump[6],0)
                        time.sleep(0.001)
                        pi.write(send[1],1)
                        pi.write(vib,1)
                        time.sleep(5)
                        pi.write(send[1],0)
                        pi.write(vib,0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reseting all variables for next drink
                    w=0 
                    stopcount=0             #end of drink 1
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0
            if x==2:         #drink 2 selected
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[1],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D2L[0] < weight < D2R[0]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(pump[2],1)                   
                    if D2L[1] < weight < D2R[1]:          
                        pi.write(pump[2],0)
                        time.sleep(0.001)
                        pi.write(pump[6],1)
                    if D2L[2] < weight < D2R[2]:         
                        pi.write(pump[6],0)
                        time.sleep(0.001)
                        pi.write(send[2],1)
                        pi.write(vib,1)
                        time.sleep(5)
                        pi.write(send[2],0)
                        pi.write(vib,0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reseting all variables
                    w=0 
                    stopcount=0             #end of drink 2
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0
            if x==3:         #drink 3 selected
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[1],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D3L[0] < weight < D3R[0]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(pump[2],1)                   
                    if D3L[1] < weight < D3R[1]:          
                        pi.write(pump[2],0)
                        time.sleep(0.001)
                        pi.write(pump[3],1)
                    if D3L[2] < weight < D3R[2]:         
                        pi.write(pump[3],0)
                        time.sleep(0.001)
                        pi.write(send[3],1)
                        pi.write(vib,1)
                        time.sleep(5)
                        pi.write(send[3],0)
                        pi.write(vib,0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reseting all variables
                    w=0 
                    stopcount=0             #end of drink 3
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0 
            if x==4:         #drink 4 selected
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[1],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D4L[0] < weight < D4R[0]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(pump[4],1)
                    if D4L[1] < weight < D4R[1]:         
                        pi.write(pump[4],0)
                        time.sleep(0.001)
                        pi.write(send[4],1)
                        pi.write(vib,1)
                        time.sleep(5)
                        pi.write(send[4],0)
                        pi.write(vib,0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reseting all variables
                    w=0 
                    stopcount=0             #end of drink 4
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0 
            if x==5:         #drink 5 selected
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[2],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D5L[0] < weight < D5R[0]:         
                        pi.write(pump[2],0)
                        time.sleep(0.001)
                        pi.write(pump[4],1)                   
                    if D5L[1] < weight < D5R[1]:          
                        pi.write(pump[4],0)
                        time.sleep(0.001)
                        pi.write(pump[1],1)
                    if D5L[2] < weight < D5R[2]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(send[5],1)
                        pi.write(vib,1)
                        time.sleep(5)
                        pi.write(send[5],0)
                        pi.write(vib,0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reseting all variables
                    w=0 
                    stopcount=0             #end of drink 5
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0 
            if x==6:         #drink 6 selected
                if w==0:
                    t=0
                    n=0
                    tare=0
                if w<51:      
                    t+=reading
                    n+=1
                    print ("t=", t)
                    #time.sleep(0.001)
                if w==52:
                    tare=t/n
                    print("tare =", tare)
                    time.sleep(1)
                if w==54:
                    pi.write(pump[1],1)
                if w>54:
                    weight=reading-tare
                    print(weight)
                    if D6L[0] < weight < D6R[0]:         
                        pi.write(pump[1],0)
                        time.sleep(0.001)
                        pi.write(send[6],1)
                        time.sleep(3)
                        pi.write(send[6],0)
                        stopcount=1
                w+=1
                if stopcount==1:            #reset all variables
                    w=0 
                    stopcount=0             #end of drink 6
                    H=0
                    t=0
                    n=0
                    tare=0
                    x=0 

        time.sleep(.01)
#!/usr/bin/python

import time, sys
import motetalk as motetalk
from motetalk import cmd, packetify, setpwm
import input_functions as inpf
import numpy as np
import scipy as sp
import quats as qt
import matplotlib.pyplot as plt
import matplotlib.axes as Axes

import math

def startup(m):
  m.sendbase(cmd.radio(int(chan)))

  m.sendheli(cmd.flags(cmd.ledmode_cnt + cmd.tick))
  m.sendheli(cmd.mode(cmd.mode_imu_loop))

  m.sendbase(cmd.flags(cmd.ledmode_cnt + cmd.notick))
  m.sendbase(cmd.mode(cmd.mode_sniff))

def end(m):
  m.sendbase(cmd.mode(cmd.mode_spin))
  m.end()

#Initialize plots
def preparePlots(drawSize):
  sensors = ["accel", "gyro", "magnet"]
  width, height = plt.figaspect(0.35)
  fig, ax = plt.subplots(nrows=3,ncols=2,sharex=True,figsize=(width,height))
  fig.set_tight_layout(True)
  p = np.zeros(drawSize)
  line1, = ax[0][0].plot(p)
  line2, = ax[1][0].plot(p)
  line3, = ax[2][0].plot(p)
  for i in range(2):
    ax[0][i].set_title("Plot x-axis " + sensors[i])
    ax[0][i].set_xlim([0,drawSize])
    ax[0][i].set_ylim([-1,1])
    ax[1][i].set_title("Plot y-axis " + sensors[i])
    ax[1][i].set_xlim([0,drawSize])
    ax[1][i].set_ylim([-1,1])
    ax[2][i].set_title("Plot z-axis " + sensors[i])
    ax[2][i].set_xlim([0,drawSize])
    ax[2][i].set_ylim([-1,1])
  plt.ion()
  fig.show()
  return fig, (line1, line2, line3) 
 
#Update all plots
def realPlot(buf, lines, fig):
  for i in range(3):
    norm = np.max(np.abs(buf[i]))
    print norm
    buf[i] /= norm
    lines[i].set_ydata(buf[i])
  plt.draw()
  plt.pause(0.01)


if __name__=="__main__":
  #initialize motetalk/serial port stuff
  sport = "/dev/tty.usbmodemfa131" #raw_input('port \n')
  chan = "15" #raw_input('Channel to sniff \n')
  filename = "data"
  log = open(filename + ".txt", 'w')
  
  m = motetalk.motetalk(sport=sport,brate=115200)
  startup(m)
  
  sys.stderr.write( "Sniffing...\n")
  try:
   (arr, t, crc) = m.nextline()
   (arr, t, crc) = m.nextline()
   (arr, t, crc) = m.nextline()
  except: 
    pass

  parse = False 
  PARSE_WIDTH = 100 
  TAIL_WIDTH = 20

  print "lets start crackin"
  #init plots
  fig, lines = preparePlots(PARSE_WIDTH)

  #time stamp
  itime = time.time()

  #main loop
  while 1:
    buf = np.zeros((10,PARSE_WIDTH))
    for i in range(PARSE_WIDTH):
      #receive packet
      (arr, t, crc) = m.nextline(parse)
      
      #timestamp
      
      #unpack packet into list
      arr = map(ord, arr[PARSE_WIDTH-TAIL_WIDTH:])
      #check if packet error: checks packet length and crc, hard-coded method
      if len(arr) > 28:
        if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
          #separates packets into values, puts things into buffer
          (buf[0][i], buf[1][i], buf[2][i], buf[3][i], buf[4][i], buf[5][i], buf[6][i], buf[7][i], buf[8][i], buf[9][i]) = inpf.preparepkt(arr)

    #get frequency
    #print time.time()-itime

    #real time plot
    realPlot(buf, lines, fig)

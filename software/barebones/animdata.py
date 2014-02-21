#!/usr/bin/python

import time, sys
import motetalk as motetalk
from motetalk import cmd, packetify, setpwm
import input_functions as inpf
import numpy as np
import scipy as sp
import quats as qt
import math
import matplotlib.pyplot as plt
import matplotlib.axes as Axes
from matplotlib import animation


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
def preparePlots(frames):
  sensors = [(sen, dim) for sen in ["accel", "gyro", "magnet"] for dim in ["x","y","x"]]
  width, height = plt.figaspect(0.35)
  fig, ax = plt.subplots(nrows=3,ncols=2,sharex=True,figsize=(width,height))
  axes = [x for ax_row in ax for x in ax_row]
  fig.set_tight_layout(True)
  p = np.zeros(frames)
  lines = [x.plot(p)[0] for x in axes]
  items = enumerate(axes)
  for i, axis in items:
    axis.set_title("Plot " + sensors[i][0]+"-axis " + sensors[i][1])
    axis.set_xlim([0,frames])
    axis.set_ylim([-1,1])
  plt.ion()
  fig.show()
  return fig, axes, lines 

def fillData(frames, norm=False):
  buf = np.zeros(0)
  for i in range(frames):
    #receive packet
    (arr, t, crc) = m.nextline(False)
    #unpack packet into list
    arr = map(ord, arr)
    #check if packet error: checks packet length and crc, hard-coded method
    if len(arr) > 28:
      if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
        #separates packets into values, puts things into buffer
        newpkt = inpf.preparepkt(arr)
        d = np.linalg.norm(newpkt)
        if norm:
          buf = np.append(buf, newpkt/d, 1)
        else:
          buf = np.append(buf, newpkt, 1)
  buf = np.reshape(buf,(frames,10))
  return np.transpose(buf) 
 
#Update all plots
def realPlot(frames, lines, degrees=1):
  print "hi"
  buf = fillData(frames, norm=True)
  for i in range(degrees):
    lines[i].set_ydata(buf[i])
  return lines[0]


if __name__=="__main__":
  #initialize motetalk/serial port stuff
  sport = "/dev/tty.usbmodemfd121" #raw_input('port \n')
  chan = "15" #raw_input('Channel to sniff \n')
  filename = "data"
  log = open(filename + ".txt", 'w')
  
  m = motetalk.motetalk(sport=sport,brate=115200)
  startup(m)
  
  sys.stderr.write( "Sniffing...\n")
  #sniff first three packets to ack receiver working
  try:
   (arr, t, crc) = m.nextline()
   (arr, t, crc) = m.nextline()
   (arr, t, crc) = m.nextline()
  except: 
    pass

  #init plots
  frames = 20 
  fig, ax, lines = preparePlots(frames)

  ani = animation.FuncAnimation(fig, realPlot, frames=frames, interval=25, fargs=(lines,), blit=False)
  plt.show()

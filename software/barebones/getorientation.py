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
import matplotlib.animation as animation
import quaternion as quat

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
def preparePlots(frames,ylim):
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
    axis.set_ylim(ylim)
  plt.ion()
  fig.show()
  return fig, axes, lines 

def getPacket(deg):
  done = False
  while (not done):
    (arr, t, crc) = m.nextline(False)
    #unpack packet into list
    arr = map(ord, arr)
    #check if packet error: checks packet length and crc, hard-coded method
    if len(arr) > 28:
      if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
        #separates packets into values, puts things into buffer
        done = True
        newpkt = inpf.preparepkt(arr)[:deg]
        return newpkt

#Fill up buffer with sensor data
def fillData(frames, deg, norm=False):
  buf = []
  for i in range(frames):
    #receive packet
    newpkt = getPacket(deg)
    buf += [newpkt,]
  buf = np.array(buf)
  buf = np.reshape(buf,(frames,deg))
  if norm:
    for row in buf:
      row /= np.linalg.norm(row)
  return buf

#Fill up buffer with euler angles data
def fillAngles(q, frames, norm=False):
  buf = []
  for i in range(frames):
    #receive packet
    newpkt = getPacket(9)
    #euler angles in order of yaw, pitch and roll
    q, euler = procQuats(q, newpkt)
    buf += [euler,]
  buf = np.concatenate(buf)
  buf = np.reshape(buf, (frames, 3))
  return buf, q

#Update all plots
def realPlot(frames, lines, degrees=1):
  #overhead = 30
  buf =  fillData(frames, degrees, norm=False)
  for i in range(degrees):
    lines[i].set_ydata(buf[:,i])
  plt.draw()

#Update all plots with quaternions orientation
def realPlotAngles(q, frames, lines):
  buf, newQuat =  fillAngles(q, frames)
  for i in range(3):
    lines[i].set_ydata(buf[:,i])
  plt.draw()
  return newQuat

#Takes in euler angles and returns euler angles as well
def procQuats(q, data):
  newQuat = quat.updateQuat(q, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])
  eulers = quat.quat2Euler(newQuat)
  return newQuat, eulers

if __name__=="__main__":
  #timestamp
  t0 = time.time()
  #initialize motetalk/serial port stuff
  sport = "/dev/tty.usbmodemfa131" #raw_input('port \n')
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

  frames = 30 
  print "lets get crackin"
  q_init = np.array([1, 0, 0, 0])
  fig, ax, lines = preparePlots(frames, [-1.5,1.5])
  plt.show()

  data = getPacket(9)
  newQuat, euler = procQuats(q_init, data)
  #Main loop
  while (1):
    #newQuat, euler = procQuats(newQuat, data)
    #realPlot(frames, lines, degrees=6)
    newQuat = realPlotAngles(newQuat, frames, lines)

    

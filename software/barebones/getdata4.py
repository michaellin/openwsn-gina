import time, sys
import motetalk as motetalk
from motetalk import cmd, packetify, setpwm
import input_functions as inpf
import numpy as np
import scipy as sp
import math
import matplotlib.pyplot as plt
import matplotlib.axes as Axes
import matplotlib.animation as animation

#multiprocessing
from multiprocessing import Process
from Queue import Queue
import threading


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
def preparePlots(frames, ylim):
  sensors = [(sen, dim) for sen in ["accel", "gyro", "magnet"] for dim in ["x","y","z"]]
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
        newpkt = inpf.preparepkt(arr)
        newpkt = newpkt[:deg]
        return newpkt


def fillData(frames, degrees, norm=False):
  buf = []
  for i in range(frames):
    #receive packet
    newpkt = getPacket(degrees)
    buf += [newpkt,]
  buf = np.array(buf)
  buf = np.reshape(buf,(frames,degrees))
  if norm:
    for row in buf:
      row /= np.linalg.norm(row)
  return buf

#Update all plots
def realPlot(frames, lines, degrees=1):
  #overhead = 30
  buf =  fillData(frames, degrees, norm=True)
  for i in range(degrees):
    lines[i].set_ydata(buf[:,i])
  plt.draw()

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
  fig, ax, lines = preparePlots(frames, [-1, 1])
  plt.show()
  while (1):
    realPlot(frames, lines, degrees=6)

  #Use as blocking queue
# q = Queue()
# p1 = threading.Thread( target=get_data, args=(frames, q))
# p2 = threading.Thread( target= realPlot, args=(frames, lines, q, 3))
# p1.start()
# p2.start()
##exec('get_data(frames,q)')
##exec('realPlot(frames, lines,q, 3)')
# time.sleep(100)
# p1.terminate()
# p2.terminate()
  #init plots
  #ani = animation.FuncAnimation(fig, realPlot, fargs=(frames, lines, 3), frames=20, interval=25, blit=False)


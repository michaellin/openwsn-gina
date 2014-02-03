#!/usr/bin/python

import time, sys
import motetalk as motetalk
from motetalk import cmd, packetify, setpwm
import input_functions as inpf
import numpy as np
import scipy as sp
import quats as qt

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
  


if __name__=="__main__":
 #initialize motetalk/serial port stuff
 sport = "/dev/tty.usbmodemfd121" #raw_input('port \n')
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
  
 parse = False;
 
 q  = [1,0,0,0]
 while 1:
  #receive packet
  (arr, t, crc) = m.nextline(parse)
  #unpack packet into list
  arr = map(ord, arr)
  #check if packet error: checks packet length and crc, hard-coded method
  if len(arr) > 28:
   if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
    #separates packets into values, puts things into caches/buffers
    (az, ay, ax, gx, gy, gz, mx, my, mz, temp) = inpf.preparepkt(arr)
    print("{0} {1} {2} {3} {4} {5} {6} {7} {8} {9}".format(ax,ay,az,gx,gy,gz,mx,my,mz,temp))

# Comment out temporarily
#   accel = inpf.pushpkt(inpf.make_array3pkt(ax,ay,az),accel)
#   gyro = inpf.pushpkt(inpf.make_array3pkt(gx,gy,gz),gyro)
#   if ((mx == 0) and (my == 0) and (mz == 0)):
#    marg= inpf.pushpkt(margpkt, marg)
#   else:
#    margpkt = inpf.make_array3pkt(mx + 0.93859,my -3.3702 ,mz - 4.3377)
#    marg = inpf.pushpkt(margpkt, marg)
#   therm = inpf.pushpkt(inpf.make_array1pkt(temp), therm)
#   time_arr = inpf.pushpkt(inpf.make_array1pkt(t), time_arr)
#   #Get initial values for quaternion recalibration
#   if (i<50):
#    i+=1
#   elif (i == 50):
#    i+=1
#    a_ini = np.sum(accel,0)/accel.shape[0]
#    mag_ini = np.sum(marg,0)/marg.shape[0]
#    (m_ini, q) = qt.initialize_q(a_ini, mag_ini)
#   else:
#    #processing data
#    #calculate quaternion
#    q = qt.quatmaker(m_ini, accel, marg, gyro, q, time_arr)
#    
#    #log.write(str(marg[25,0]) + " " + str(marg[25,1]) + " " + str(marg[25,2]) + "\n")
#    
#    roll = math.atan2(2*(q[0]*q[1] + q[2] * q[3]),(1-2*(q[1]**2 + q[2] **2)))
#    pitch = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
#    yaw = math.atan2(2*(q[0]*q[3] + q[1] * q[2]),(1-2*(q[2]**2 + q[3] **2)))
#    
#    axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
#    up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
#    platform.axis=axis
#    platform.up=up
#    platform.length=1.0
#    platform.width=0.65
#    plat_arrow.axis=axis
#    plat_arrow.up=up
#    plat_arrow.length=0.8
#    p_line.axis=axis
#    p_line.up=up
#    cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
#    cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
#    cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
#    cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
#    arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)

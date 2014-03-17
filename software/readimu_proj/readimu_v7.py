#!/usr/bin/python

#This readimu is used for taking data with just the one ring (to rule them all)
#Additoinally it attempts to follow and plot where the ring is pointing in space using the PyVisual

import time, sys
import lib.motetalk as motetalk
from lib.motetalk import cmd, packetify, setpwm
from datetime import datetime
from visual import *
import math

def startup(m,chan):
  m.sendbase(cmd.radio(chan))
  # time.sleep(1)
  m.sendbase(cmd.flags(cmd.ledmode_cnt + cmd.notick))
  # time.sleep(1)

  m.sendheli(cmd.flags(cmd.ledmode_cnt + cmd.tick))
  #time.sleep(1)
  m.sendheli(cmd.mode(cmd.mode_imu_loop))
  #time.sleep(1)

  m.sendbase(cmd.mode(cmd.mode_sniff))
  # time.sleep(1)


#Quaterneons
q0 = [1,1]
q1 = [0,0]
q2 = [0,0]
q3 = [0,0]

def Process_Gyro(ti,gx,gy,gz):
#This assumes we are using C3 and that the linear regression factors are as follows
	gx_tc = [0.0114,-624.9361]
	gy_tc = [-0.0088,451.4147]
	gz_tc = [-0.0106,542.6805]
	ts = .003 #This is the time step...could be added in programatically
	#We will now temperature correct the gyro values
	gx = gx - gx_tc[0]*ti-gx_tc[1]
	gy = gy - gy_tc[0]*ti-gy_tc[1]
	gz = gz - gz_tc[0]*ti-gz_tc[1]
	#we will now convert to radians and scale from data sheet value
	gx = math.radians(gx/14.375)
	gy = math.radians(gy/14.375)
	gz = math.radians(gz/14.375)
	#Now make quaterneons
	q0dot = -q1[0]*gx/2.0-q2[0]*gy/2.0-q3[0]*gz/2.0
	q1dot = q0[0]*gx/2.0+q2[0]*gy/2.0-q3[0]*gz/2.0
	q2dot = q0[0]*gy/2.0-q1[0]*gy/2.0+q3[0]*gz/2.0
	q3dot = q0[0]*gx/2.0+q1[0]*gy/2.0-q2[0]*gz/2.0
	#The previous data is used to determine the new data
	q0[1] = q0[0] + q0dot*ts
	q1[1] = q1[0] + q1dot*ts
	q2[1] = q2[0] + q2dot*ts
	q3[1] = q3[0] + q3dot*ts
	#the quaterneons are now normalized, but I'm not sure it does anything. 
	n = math.sqrt(q0[1]*q0[1]+q1[1]*q1[1]+q2[1]*q2[1]+q3[1]*q3[1])
	q0[1] = q0[1]/n 
	q1[1] = q1[1]/n
	q2[1] = q2[1]/n
	q3[1] = q3[1]/n
	#The angles are now determined
	theta =  math.atan2(2*(q0[1]*q1[1] + q2[1]*q3[1]),1 - 2*(q1[1]*q1[1] + q2[1]*q2[1]))
	phi  =  math.asin(2 * (q0[1]*q2[1] - q1[1]*q3[1]))
	psi = math.atan2(2 * (q0[1]*q3[1] + q1[1]*q2[1]), 1 - 2 *(q2[1]*q2[1] + q3[1]*q3[1]))
	#The quaterneons are not shifted, i.e the new q values are moved to the old q value spot
	q0[0]=q0[1]
	q1[0]=q1[1]
	q2[0]=q2[1]
	q3[0]=q3[1]
	return phi,theta

def end(m):
  m.sendbase(cmd.mode(cmd.mode_spin))
  m.end()

def move_E(obj,theta,phi):
	Mag = obj.axis.mag
	x= cos(phi)*sin(theta)
	y= sin(phi)*sin(theta)
	z= cos(theta)
	obj.axis = (Mag*x,Mag*z,Mag*y)

def move_C(obj,x,z,y):
	Mag = obj.axis.mag
	vec = vector(x,z,y)
	vec2 = vec.norm() #This is a unit vector
	obj.axis = (Mag*vec2.x,Mag*vec2.z,Mag*vec2.y)

ring = arrow(pos=(0,0,0), axis=(0,-4.7,0), shaftwidth=.3,color = color.white)
pad = arrow(pos=(0,0,0), axis=(0,5,0), shaftwidth=.2,color = color.black)

offset = -4
s = 20
distant_light(direction=(1,1,1))

ball = sphere(pos = (0,0,0),radius = 5, color=color.red, opacity=0.4)


floor = box (pos=(0-.5*offset,0+2*offset,0+offset), length=s, height=0.5, width=s, color=color.blue)
wall = box (pos=(0-.5*offset,s/2+2*offset,-s/2+offset), length=s, height=s, width=.5, color=color.green)
wall2 = box (pos=(s/2-.5*offset,s/2+2*offset,0+offset), length=.5, height=s, width=s, color=color.yellow)


chan1 = 15
com1 = "/dev/tty.usbmodemfd121" #"COM19"
  
num_good = 0
num_bad = 0
num_skip = 0

done = 0
n = 0
oldarr = []

header =    "addr len cmd type n sx sy z1 z3 ta mx my mz lx ly lz ti gx gy gz mychk chk lqi rssi"
fmtstr = '!  H    b   b   b    H H  H  H  H  H  h  h  h  H  H  H  H  h  h  h  B     H   B   B'
# header =    "addr  len   cmd type   n mx my mz   lx ly lz   ti gx gy gz   cm ct ca ce cp   i0 i1 i2 i3 i4 i5 i6 i7 i8 i9 ia ib ic mychk chk lqi rssi"
# fmtstr = '!  H       b   b   b      H h  h  h     H  H  H    H  h  h  h    H  H  H  H  H   B  B  B  B  B  B  B  B  B  B  B  B  B  B     H   B   B'
header =    "addr len cmd type n sx sy z1 z3 ta mx my mz lx ly lz ti gx gy gz cm ct ca ce cp mychk chk lqi rssi"
fmtstr = '!  H    b   b   b    H H  H  H  H  H  h  h  h  H  H  H  H  h  h  h  h  h  h  h  h  B     H   B   B'

#Joey's Header
header = "addr len cmd type n sx sy z1 z3 ta lx ly lz ti gx gy gz Address"
fmtstr = '! H  b   b   b  H  H  H  H  H  H  H  H  H  H  h  h  h H'

#Joey's Header
header = "addr len cmd type n sx sy z1 z3 ta lx ly lz ti gx gy gz mx my mz Address"
fmtstr = '! H  b   b   b  H  H  H  H  H  H  H  H  H  H  h  h  h h h h H'


m1 = motetalk.motetalk(fmtstr, header, com1, debug=False, timeout = .5)
startup(m1,chan1) #Channel 15

sys.stderr.write( "Sniffing...\n")
print "ts " + header + " Time"

try:
  (arr, t, crc) = m1.nextline()
  sys.stderr.write(com1)
  (arr, t, crc) = m1.nextline()
  sys.stderr.write(com1)
  (arr, t, crc) = m1.nextline()
  sys.stderr.write(com1)
  
except: 
  sys.stderr.write("oops")
  pass

parse = False 
PARSE_WIDTH = 20 

print "let's start visualizin'"


#This will not draw the things
while not done:
   try:
    (arr, t, crc) = m1.nextline(parse)
    arr = map(ord, arr)
    print arr
########if(a[0]!='ts'):
########	 	phi,theta = Process_Gyro(int(a[14]),int(a[15]),int(a[16]),int(a[17]))
########	 	move_E(pad,theta,phi)
########	 	move_C(ring,int(a[6])-2048,-1*(int(a[7])-2048),int(a[8])-2048)
########	 	rate(1000)

   except:
    sys.stderr.write(repr(sys.exc_info()))
    sys.stderr.write("\nQuitting... \n")
    done = 1

end(m1)
sys.stderr.write( "%d Successful packets recieved\n" % num_good)
sys.stderr.write( "%d packet errors\n" % num_bad)
sys.stderr.write( "%d skipped packets\n" % num_skip)
if (num_skip + num_good):
  sys.stderr.write( "%4.2f%% packet drop rate\n" % (num_skip * 100.0 / (num_skip + num_good)))


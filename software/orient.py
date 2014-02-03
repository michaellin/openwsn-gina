#!/usr/bin/python

import time, sys
import motetalk as motetalk
from motetalk import cmd, packetify, setpwm
import input_functions as inpf
import numpy as np
import scipy as sp
import process_pos as pos
import quats as qt

from visual import *
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
	# Main scene
	scene=display(title="9DOF Ring GINA test")
	scene.range=(1.2,1.2,1.2)
	scene.forward = (0,-1,-0.25)
	#scene.forward = (1,0,-0.25)
	scene.up=(0,0,1)
	
	# Second scene (Roll, Pitch, Yaw)
	scene2 = display(title='9DOF Ring GINA test',x=0, y=0, width=500, height=200,center=(0,0,0), background=(0,0,0))
	scene2.range=(1,1,1)
	scene.width=500
	scene.y=200
	
	scene2.select()
	#Roll, Pitch, Yaw
	cil_roll = cylinder(pos=(-0.4,0,0),axis=(0.2,0,0),radius=0.01,color=color.red)
	cil_roll2 = cylinder(pos=(-0.4,0,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
	cil_pitch = cylinder(pos=(0.1,0,0),axis=(0.2,0,0),radius=0.01,color=color.green)
	cil_pitch2 = cylinder(pos=(0.1,0,0),axis=(-0.2,0,0),radius=0.01,color=color.green)
	#cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
	#cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
	arrow_course = arrow(pos=(0.6,0,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)
	
	#Roll,Pitch,Yaw labels
	label(pos=(-0.4,0.3,0),text="Roll",box=0,opacity=0)
	label(pos=(0.1,0.3,0),text="Pitch",box=0,opacity=0)
	label(pos=(0.55,0.3,0),text="Yaw",box=0,opacity=0)
	label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=color.yellow)
	label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=color.yellow)
	label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=color.yellow)
	label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=color.yellow)
	label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=color.yellow)
	label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=color.yellow)
	label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=color.yellow)
	label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=color.yellow)
		
	# Main scene objects
	scene.select()
	# Reference axis (x,y,z)
	arrow(color=color.green,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
	arrow(color=color.green,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
	arrow(color=color.green,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)
	# labels
	label(pos=(0,0,0.8),text="9DOF GINA test",box=0,opacity=0)
	label(pos=(1,0,0),text="X",box=0,opacity=0)
	label(pos=(0,-1,0),text="Y",box=0,opacity=0)
	label(pos=(0,0,-1),text="Z",box=0,opacity=0)
	# IMU object
	platform = box(length=1, height=0.05, width=1, color=color.red)
	p_line = box(length=1,height=0.08,width=0.1,color=color.yellow)
	plat_arrow = arrow(color=color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)

	#initialize motetalk/serial port stuff
	sport = "/dev/tty.usbmodemfa131" #raw_input('port \n')
	chan = "15" #raw_input('Channel to sniff \n')
	#filename = raw_input('Log file to write to \n')
	filename = "testroll"
	log = open(filename + ".txt", 'w')
	#log2 = open(filename + "2.txt", 'w')
	
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
	
	i=0
	
	#initialize everything here
	accel = inpf.array3_init()
	gyro = inpf.array3_init()
	marg = inpf.array3_init()
	therm = inpf.array1_init()
	time_arr = inpf.array1_init()

	tstart  = 0
	axstart = 0
	aystart = 0
	azstart = 0
	vx      = 0
	vy      = 0
	vz      = 0
	x       = 0
	y       = 0
	z       = 0
	state   = 0
	p		= 0
	q		= [1,0,0,0]
	margpkt = inpf.make_array3pkt(0,0,0)
	j = 0
	while 1:
		rate (2000)
		#receive packet
		(arr, t, crc) = m.nextline(parse)
		#unpack packet into list
		arr = map(ord, arr)
		#check if packet error: checks packet length and crc, hard-coded method
		if len(arr) > 28:
			if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
				#separates packets into values, puts things into caches/buffers
				(az, ay, ax, gx, gy, gz, mx, my, mz, temp) = inpf.preparepkt(arr)
				accel = inpf.pushpkt(inpf.make_array3pkt(ax,ay,az),accel)
				gyro = inpf.pushpkt(inpf.make_array3pkt(gx,gy,gz),gyro)
				if ((mx == 0) and (my == 0) and (mz == 0)):
					marg= inpf.pushpkt(margpkt, marg)
				else:
					margpkt = inpf.make_array3pkt(mx + 0.93859,my -3.3702 ,mz - 4.3377)
					marg = inpf.pushpkt(margpkt, marg)
				therm = inpf.pushpkt(inpf.make_array1pkt(temp), therm)
				time_arr = inpf.pushpkt(inpf.make_array1pkt(t), time_arr)
				#Get initial values for quaternion recalibration
				if (i<50):
					i+=1
				elif (i == 50):
					i+=1
					a_ini = np.sum(accel,0)/accel.shape[0]
					mag_ini = np.sum(marg,0)/marg.shape[0]
					(m_ini, q) = qt.initialize_q(a_ini, mag_ini)
				else:
					#processing data
					#calculate quaternion
					q = qt.quatmaker(m_ini, accel, marg, gyro, q, time_arr)
					
					#log.write(str(marg[25,0]) + " " + str(marg[25,1]) + " " + str(marg[25,2]) + "\n")
					
					roll = math.atan2(2*(q[0]*q[1] + q[2] * q[3]),(1-2*(q[1]**2 + q[2] **2)))
					pitch = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
					yaw = math.atan2(2*(q[0]*q[3] + q[1] * q[2]),(1-2*(q[2]**2 + q[3] **2)))
					
					axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
					up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
					platform.axis=axis
					platform.up=up
					platform.length=1.0
					platform.width=0.65
					plat_arrow.axis=axis
					plat_arrow.up=up
					plat_arrow.length=0.8
					p_line.axis=axis
					p_line.up=up
					cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
					cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
					cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
					cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
					arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)

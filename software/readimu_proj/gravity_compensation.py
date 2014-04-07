#!/usr/bin/python

import time, sys
import lib.motetalk as motetalk
from lib.motetalk import cmd, packetify, setpwm
import lib.input_functions as inpf
import numpy as np
import scipy as sp
import lib.quats as qt
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
		axis.set_ylim([-10,10])
	plt.ion()
	fig.show()
	return fig, axes, lines 

# From: http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
# compensate the accelerometer readings from gravity. 
# @param q the quaternion representing the orientation of a 9DOM MARG sensor array
# @param acc the readings coming from an accelerometer expressed in g
#
# @return a 3d vector representing dinamic acceleration expressed in g
def gravity_compensate(q, acc):
	g = [0.0, 0.0, 0.0]
	
	# get expected direction of gravity
	g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
	g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
	g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
	
	# compensate accelerometer readings with the expected direction of gravity
	return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

def fillData(frames, norm=False):
	buf = np.zeros(0)
	for i in range(frames):
		#receive packet
		done = 0
		while not done:
			(arr, t, crc) = m.nextline(False)
			#unpack packet into list
			arr = map(ord, arr)
			#check if packet error: checks packet length and crc, hard-coded method
			if len(arr) > 28:
				if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
					done = 1
					#separates packets into values, puts things into buffer
					newpkt = inpf.preparepkt(arr)[:3]
					print newpkt
					buf = np.append(buf, newpkt, 1)
	buf = np.reshape(buf,(frames,3))
	for row in buf:
		row /= np.linalg.norm(row)
	return buf 

def fillDataGravity(frames, norm=False):
	buf = np.zeros(0)
	#global variable: GRAVITY - will update after obtaining new packet
	gravity = [0, 0, 0]
	for i in range(frames):
		#receive packet
		done = 0
		while not done:
			(arr, t, crc) = m.nextline(False)
			#unpack packet into list
			arr = map(ord, arr)
			#check if packet error: checks packet length and crc, hard-coded method
			if len(arr) > 28:
				if ((arr[22] * 256 + arr[23]) == (sum(arr) - arr[22] - arr[23] - arr[24] - arr[25] - arr[26] - arr[27] - arr[28])):
					done = 1
					#USING ANDROID SENSOR IMPLEMENTATION
					accel = inpf.preparepkt(arr)[:3]
					difference, newaccelpkt = gravity_elim(accel, gravity)
					buf = np.append(buf, newaccelpkt, 1)
					# print "raw: ", accel
					# print "no_g: ", newaccelpkt
					# print "diff: "
					# print difference

					gyro = inpf.preparepkt(arr)[3:6]
					#buf = np.append(buf, gyro, 1)
					
					"""
					# USING QUATERNIONS from "quats.py"
					#separates packets into values, puts things into buffer
					#initalize all 9DOF
					accel = inpf.preparepkt(arr)[:3]
					gyro = inpf.preparepkt(arr)[3:6]
					mag = inpf.preparepkt(arr)[6:9]
					#initialize quaternions
					m_ini, q = qt.initialize_q(accel, mag)
					#determine overall current orientation of quaternion
					#print accel, gyro, mag
					quat = qt.quatmaker(m_ini, accel, mag, gyro, q, t)
					#creates new accelerometer array with gravity comepnsated
					newaccelpkt = gravity_compensate(quat, accel)
					buf = np.append(buf, newaccelpkt, 1)

					#print newaccelpkt
					"""
	buf = np.reshape(buf,(frames,3))
	# for row in buf:
	# 	row /= np.linalg.norm(row)
	return buf 

#Deprecated
def get_data(frames):
	buf = fillData(frames, norm=True)
	return buf
 
#Update all plots
def realPlot(frames, lines, degrees=1):
	#overhead = 30
	buf =  fillData(frames,norm=True)
	for i in range(degrees):
		lines[i].set_ydata(buf[:,i])
	plt.draw()

def realPlotGravity(frames, lines, degrees=1):
	buf =  fillDataGravity(frames,norm=True)
	for i in range(degrees):
		lines[i].set_ydata(buf[:,i])
	plt.draw()

# Adapted from: http://developer.android.com/reference/android/hardware/SensorEvent.html
def gravity_elim(accel, gravity):
	"""
	*readimu_v6 prints arrays of len = 21 --> gx = index 15; gy = index 16; gz = index 17
	*alpha is calculated as t / (t + dT) --> t = LPF's time-constant; dT = sampling/delivery rate
	"""
	lin_acc = [0, 0, 0]

	#convert acceleration data as an array
	a = []
	a.append(accel)
	a = np.array(a)
	a.flatten()

	alpha = 0.8

	gravity[0] = alpha * gravity[0] + (1-alpha) * (accel[0]-gravity[0]) #gx
	gravity[1] = alpha * gravity[1] + (1-alpha) * (accel[1]-gravity[1]) #gy
	gravity[2] = alpha * gravity[2] + (1-alpha) * (accel[2]-gravity[2]) #gz

	lin_acc[0] = (accel[0]-gravity[0]) - gravity[0]
	lin_acc[1] = (accel[1]-gravity[1]) - gravity[1]
	lin_acc[2] = (accel[2]-gravity[2]) - gravity[2]

	diff = a-lin_acc

	return tuple(diff), tuple(lin_acc)

if __name__=="__main__":
	#timestamp
	t0 = time.time()
	#initialize motetalk/serial port stuff
	sport = "/dev/tty.usbmodemfa131" #raw_input('port \n')
	chan = "15" #raw_input('Channel to sniff \n')
	#filename = "data"
	#filename = raw_input('Log file to write to \n')
	#log = open(filename + ".txt", 'w')


	m = motetalk.motetalk(sport=sport,brate=115200)
	startup(m)
	
	sys.stderr.write( "Sniffing...\n")
	try:
	 (arr, t, crc) = m.nextline()
	 (arr, t, crc) = m.nextline()
	 (arr, t, crc) = m.nextline()
	except: 
		pass

frames = 50 
print "lets get crackin"
fig, ax, lines = preparePlots(frames)
plt.show()

while (1):
	#realPlot(frames, lines, degrees=3)
	realPlotGravity(frames, lines, degrees=3)

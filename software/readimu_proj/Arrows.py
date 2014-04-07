'''
green-blue = x axis
green-yellow = z-axis
yellow-blue = y-axis

NOTE!!!! All vectors are indexed vec(x-axis,z-axis,y-axis)
'''
from visual import *
import math, sys, time

def rot(ring,pad):
	ring.rotate(angle=dt*pi/4,axis = (0,1,0))
	pad.rotate(angle=dt*pi/4,axis = (1,0,0))
	

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


file = open("output3.txt")

opsi = []
ophi = []
otheta = []

while 8:
	line = file.readline()
	if not line:
		break
	a = line.split()
	opsi.append(math.radians(float(a[0])))
	ophi.append(math.radians(float(a[1])))
	otheta.append(math.radians(float(a[2])))


ring = arrow(pos=(0,0,0), axis=(4.7,0,0), shaftwidth=.3,color = color.white)
pad = arrow(pos=(0,0,0), axis=(5,0,0), shaftwidth=.2,color = color.black)

offset = -4
s = 20
distant_light(direction=(1,1,1))

ball = sphere(pos = (0,0,0),radius = 5, color=color.red, opacity=0.4)


floor = box (pos=(0-.5*offset,0+2*offset,0+offset), length=s, height=0.5, width=s, color=color.blue)
wall = box (pos=(0-.5*offset,s/2+2*offset,-s/2+offset), length=s, height=s, width=.5, color=color.green)
wall2 = box (pos=(s/2-.5*offset,s/2+2*offset,0+offset), length=.5, height=s, width=s, color=color.yellow)


dt = 0.01
count = 0
var = True

theta = 0
phi=0

x = 1
y = 1
z = 1

count2 = 0

scene.exit = False


while var:
	rate(100)
	if scene.kb.keys:
		print scene.kb.getkey(), 
	else:
		count2 +=50
		count +=1
		x = sin(count/100.0)
		y = cos(count/100.0)
		z = 0
		rate(100) #no more than 100 frames per second on a fast computer
		move_C(ring,x,y,z)
		move_E(pad,otheta[count],ophi[count])

		#sys.stderr.write(repr(sys.exc_info()))
		#sys.stderr.write("\nQuitting... \n")
		#done = 1

#print "hey"

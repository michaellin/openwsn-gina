#Packet management functions: sets up buffers, caches, arrays, and scales all values appropriately.

import numpy as np

def getvalues(a,b):
	if (((a*256 + b) & (1<<15))!= 0):
		return ((a*256 + b) - (1<<16))
	else:
		return (a*256 + b)

def accscale(a):
	return a*39.2266/65535

####
# Gyroscope Full-Scale Range FS_SEL=3 +/- 2000 deg/s
# Sensitivity factor FSSEL=1 65.5 LSB/(deg/s)
def gyroscale(a):
	return a*np.pi*2000./(65535*180)

def magscale(a):
	return a*2452./65535
	
def tempscale(a):
	return a/340. + 35
	
def array3_init():
	return np.zeros(shape = (40,3))
	
def array1_init():
	return np.zeros(shape = (40,1))

	
def preparepkt(arr):
	ax = accscale(getvalues(arr[2],arr[3]))
	ay = accscale(getvalues(arr[4],arr[5]))
	az = accscale(getvalues(arr[6],arr[7]))
	temp = tempscale(getvalues(arr[8], arr[9]))
	gx = gyroscale(getvalues(arr[10],arr[11]))
	gy = gyroscale(getvalues(arr[12],arr[13]))
	gz = gyroscale(getvalues(arr[14], arr[15]))
	mx = magscale(getvalues(arr[17] ,arr[16]))
	my = magscale(getvalues(arr[19] ,arr[18]))
	mz = magscale(getvalues(arr[21] ,arr[20]))
	return np.array([ax, ay, az, gx, gy, gz, mx, my, mz, temp])
	
def make_array3pkt(v1, v2, v3):
	return np.array([[v1, v2, v3]])
	
def make_array1pkt(v1):
	return np.array([[v1]])
	
def pushpkt(pkt, arr):
	arr = np.append(arr,pkt,0)
	arr = np.delete(arr,0,0)
	return arr

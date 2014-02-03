#Position calculation functions

import numpy as np
import scipy as sp
import quats as qt


def posmaker(accel, gyro, time, tstart, axstart, aystart, azstart, vx, vy, vz, x, y, z, q):
	#rotate values according to quaternion, and split array up into respective parts
	a=qt.quatrot(q,[accel[25,0],accel[25,1],accel[25,2]])
	accel[25,0]=a[0]
	accel[25,1]=a[1]
	accel[25,2]=a[2]
	ax = accel[:,0]
	ay = accel[:,1]
	az = accel[:,2]
	gz = gyro[:,2]
	
	#Hack to pass through velocity values
	vxstart = vx
	vystart = vy
	vzstart = vz
	
	#Double integral with zvc
	if (tstart == 0):
		if (np.absolute(gz[25]) > 0.058):
			tstart = time[25]
			axstart = ax[20]
			aystart = ay[20]
			azstart = az[20]
		elif (gz[25] <0.1):
			vx = 0
			vy = 0
			vz = 0
	elif (tstart>0):
		if (min(np.absolute(gz[25:35]) < 0.058)):
			vx = vx - 0.5*(ax[25] - axstart)*(time[25] - tstart)
			vy = vy - 0.5*(ay[25] - aystart)*(time[25] - tstart)
			vz = vz - 0.5*(az[25] - azstart)*(time[25] - tstart)
			x = x -  0.5 *(vx) *(time[25] - tstart) - (ax[25] - axstart)*((time[25]-tstart)**2)/6
			y = y -  0.5 *(vy) *(time[25] - tstart) - (ay[25] - aystart)*((time[25]-tstart)**2)/6
			z = z -  0.5 *(vz) *(time[25] - tstart) - (az[25] - azstart)*((time[25]-tstart)**2)/6
			vx = 0
			vy = 0
			vz = 0
			tstart = 0
		else:
			vx = vx + (ax[25] + ax[24] - 2*axstart)*(time[25]-time[24])/2
			vy = vy + (ay[25] + ay[24] - 2*aystart)*(time[25]-time[24])/2
			vz = vz + (az[25] + az[24] - 2*azstart)*(time[25]-time[24])/2
			x = x + (vx+vxstart)*(time[26]-time[25])/2
			y = y + (vy+vystart)*(time[26]-time[25])/2
			z = z + (vz+vzstart)*(time[26]-time[25])/2
	return (tstart, axstart, aystart, azstart, vx, vy, vz, x, y, z, q)
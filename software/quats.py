#Orientation calculation functions

import numpy as np
import math

def quatmult(q,r):
	q0 = r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3]
	q1 = r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2]
	q2 = r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1]
	q3 = r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]
	return [q0,q1,q2,q3];

def conju(q):
	return [q[0],-q[1],-q[2],-q[3]]

def quatrot(q,r):
	p=[0,r[0],r[1],r[2]]
	t=quatmult(quatmult(q,p),conju(q))
	return t[1:4]

def norm(v):
	i=0
	n=0
	while i<len(v):
		n+=v[i]**2
		i+=1
	if (n != 0):
		return [x/math.sqrt(n) for x in v]
	else:
		return v

#setq is the function used to recalibrate according to mag and accel
def setq(accel, mag, m_ini):
	m = norm([mag[1], mag[0],-mag[2]])
	mo = norm([1,0,0])
	#mo = norm(m_ini)
	a = norm([accel[0], accel[1], accel[2]])
	ao = norm([0, 0, -1])
	alpha=math.acos(np.dot(np.array(a),np.array(ao)));
	n=norm((np.cross(np.array(a),np.array(ao))).tolist())
	q2=norm([np.cos(alpha/2),n[0]*np.sin(alpha/2),n[1]*np.sin(alpha/2),n[2]*np.sin(alpha/2)]);
	alpha_m=math.acos(np.dot(m,mo));
	n=norm((np.cross(np.array(m),np.array(mo))).tolist())
	q1=norm([np.cos(alpha_m/2),n[0]*np.sin(alpha_m/2),n[1]*np.sin(alpha_m/2),n[2]*np.sin(alpha_m/2)]);
	return quatmult(q2,q1);

def initialize_q(accel, mag):
	a = norm([accel[0], accel[1], accel[2]])
	ao = norm([0, 0, -1])
	alpha=math.acos(np.dot(np.array(a),np.array(ao)))
	n=norm((np.cross(np.array(a),np.array(ao))).tolist())
	q=norm([np.cos(alpha/2),n[0]*np.sin(alpha/2),n[1]*np.sin(alpha/2),n[2]*np.sin(alpha/2)])
	m = norm([mag[1],mag[0], -mag[2]])
	m_ini = quatrot(q, m)
	return m_ini, q

	
#changequat is used to rotate our quaternion according to gyroscope values
def changequat(gyro, t, q):
	gyroX = gyro[25,0]
	gyroY = gyro[25,1]
	gyroZ = gyro[25,2]
	ts = t[25]-t[24]
	
	q0dot = (-q[1]*gyroX/2 - q[2]*gyroY/2 - q[3]*gyroZ/2);
	q1dot = ( q[0]*gyroX/2 + q[2]*gyroZ/2 - q[3]*gyroY/2);
	q2dot = ( q[0]*gyroY/2 - q[1]*gyroZ/2 + q[3]*gyroX/2);
	q3dot = ( q[0]*gyroZ/2 + q[1]*gyroY/2 - q[2]*gyroX/2);
	
	q0=q[0]+q0dot*ts
	q1=q[1]+q1dot*ts
	q2=q[2]+q2dot*ts
	q3=q[3]+q3dot*ts
	
	q=norm([q0,q1,q2,q3])
	
	return q;

#quatmaker is function that determines overall current orientation
def quatmaker(m_ini, accel, marg, gyro, q, t):
	alpha = 0
	accel_avg = np.sum(accel[0:20],0)/20
	marg_avg = np.sum(marg[0:20],0)/20
	print marg_avg
	qa = setq(accel_avg, marg_avg, m_ini)
	qg = changequat(gyro, t, q)
	q0 = alpha*qa[0] + (1-alpha)*qg[0]
	q1 = alpha*qa[1] + (1-alpha)*qg[1]
	q2 = alpha*qa[2] + (1-alpha)*qg[2]
	q3 = alpha*qa[3] + (1-alpha)*qg[3]
	return norm([q0, q1, q2, q3])
#!/usr/bin/python
###########################################################################################
## Adapted from implementation of Madgwick's IMU and AHRS algorithms by Michael Lin.
## See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
##
## Date     Author          Notes
## 29/09/2011 SOH Madgwick    Initial release
## 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
## 19/02/2012 SOH Madgwick  Magnetometer measurement is normalised
##
###########################################################################################

import numpy as np
import math

# Variables
SAMPLE_FREQ = 512.0
BETA = 0.1

# Update quaternion using 9-axis sensor readings
# Input: old quaternion, and 9 axis of intertial data
# Output: new quaternion
def updateQuat(q, ax, ay, az, gx, gy, gz, mx, my, mz):
  
  #If magnetometer data not available call different measurement
  if ((mx == 0.0) and (my == 0.0) and (mz == 0.0)):
    return updateQuatNoMag(q, gx, gy, gz, ax, ay, az)
  
  #Rate of change of quaternion form gyroscope
  qDot1 = 0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz)
  qDot2 = 0.5 * (q[0]*gx + q[2]*gz - q[3]*gy)
  qDot3 = 0.5 * (q[0]*gy - q[1]*gz + q[3]*gx)
  qDot4 = 0.5 * (q[0]*gz - q[1]*gy + q[2]*gx)

  # Compute feedback parameters
  if (not ((ax == 0.0) and (ay == 0.0) and (az == 0.0))):
    #Normalize parameters
    norm_a = (ax**2 + ay**2 + az**2)**-1
    ax *= norm_a
    ay *= norm_a
    az *= norm_a
  
    norm_m = (mx**2 + my**2 + mz**2)**-1
    mx *= norm_m
    my *= norm_m
    mz *= norm_m

    #Precomputed variables
    _2q0mx = 2.0 * q[0] * mx
    _2q0my = 2.0 * q[0] * my
    _2q0mz = 2.0 * q[0] * mz
    _2q1mx = 2.0 * q[1] * mx
    _2q0 = 2.0 * q[0]
    _2q1 = 2.0 * q[1]
    _2q2 = 2.0 * q[2]
    _2q3 = 2.0 * q[3]
    _2q0q2 = 2.0 * q[0] * q[2]
    _2q2q3 = 2.0 * q[2] * q[3]
    q0q0 = q[0] **2
    q0q1 = q[0] * q[1]
    q0q2 = q[0] * q[2]
    q0q3 = q[0] * q[3]
    q1q1 = q[1] **2
    q1q2 = q[1] * q[2]
    q1q3 = q[1] * q[3]
    q2q2 = q[2] **2
    q2q3 = q[2] * q[3]
    q3q3 = q[3] * q[3]

    #Reference direction of Earth using mag data
    hx = (mx * q0q0) - (_2q0my * q[3]) + (_2q0mz * q[2]) + (mx * q1q1) + (_2q1 * my * q[2]) + (_2q1 * mz * q[3]) - (mx * q2q2) - (mx * q3q3)
    hy = (_2q0mx * q[3]) + (my * q0q0) - (_2q0mz * q[1]) + (_2q1mx * q[2]) - (my * q1q1) + (my * q2q2) + (_2q2 * mz * q[3]) - (my * q3q3)
    _2bx = np.sqrt(hx **2 + hy **2)
    _2bz = (_2q0mx * q[2]) + (_2q0my * q[1]) + (mz * q0q0) + (_2q1mx * q[3]) - (mz * q1q1) + (_2q2 * my * q[3]) - (mz * q2q2) + (mz * q3q3)
    _4bx = 2.0 * _2bx
    _4bz = 2.0 * _2bz


    #Gradients of decent (main algorithm combining all 9 axis measurements)
    s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + \
         (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + \
          _2bz * (0.5 - q1q1 - q2q2) - mz)
    s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[1] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * \
         q[3] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)+ \
         (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[2] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + \
         (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * \
         (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
    s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5 - q2q2 - q3q3) +\
         _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + \
          _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

    #Normalize the gradients
    norm_s = (s0 **2 + s1 **2 + s2 **2 + s3 ** 2) **-1
    s0 *= norm_s
    s1 *= norm_s
    s2 *= norm_s
    s3 *= norm_s

    #Apply our feedback
    qDot1 -= BETA * s0
    qDot2 -= BETA * s1
    qDot3 -= BETA * s2
    qDot4 -= BETA * s3
  
  #Integrate the rate of change to yield new quaternion
  q0 = q[0] + qDot1 *(1.0/SAMPLE_FREQ)
  q1 = q[1] + qDot2 *(1.0/SAMPLE_FREQ)
  q2 = q[2] + qDot3 *(1.0/SAMPLE_FREQ)
  q3 = q[3] + qDot4 *(1.0/SAMPLE_FREQ)

  #Normalize quaternions
  norm_q = (q0 **2 + q1 **2 + q2 **2 + q3**2)**-1
  q0 *= norm_q
  q1 *= norm_q
  q2 *= norm_q
  q3 *= norm_q

  return np.array([q0, q1, q2, q3])

def quat2Euler(q):
  a = q[0]
  b = q[1]
  c = q[2]
  d = q[3]
  yaw = math.atan2(2*(a*b+c*d), (a**2-b**2-c**2+d**2))
  pitch = -np.arcsin(2*(b*d-a*c))
  roll = math.atan2(2*(a*d+b*c), (a**2+b**2-c**2-d**2))
  return [yaw, pitch, roll]

#! /usr/bin/env python
import math
import rospy
import time
from time import time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu



def proceso(msg):
    #allez = Imu()
    print "------------------------------------------------"
    print "aceleracion linear x = " + str(msg.linear_acceleration.x)
    print "aceleracion linear y = " + str(msg.linear_acceleration.y)
    print "aceleracion linear z = " + str(msg.linear_acceleration.z)
    #rate.sleep()  #-->  Si lo descomentamos obtendremos lecturas antiguas, se leera todo el historial de /imu 
    #                  desde que este archivo fue corrido.

rospy.init_node('sphero_monitor') # the original name sphero might be the same as other node.
sub_imu = rospy.Subscriber('imu', Imu, proceso)
rate = rospy.Rate(5) # --> Esto nos permite controlar la velocidad a la que se leen los datos publicados en /imu
while not rospy.is_shutdown():

		"""
		#----------Calculate delta time
		ax = 0     # remplazar por datos reales
		ay = 0
		az = 0
		gx = 0
		gy = 0
		gz = 0
		q0 = 0.1 #W
		q1 = 0.1 #X
		q2 = 0.1 #Y
		q3 = 0.1 #Z
		t = time()
		currenttime = 0
		previoustime = currenttime
		currenttime = 1000000 * t + t / 1000000
		dt = (currenttime - previoustime) / 1000000.0
		if (dt < (1/1300.0)) : 
			time.sleep((1/1300.0 - dt) * 1000000)
		t = time()
		currenttime = 1000000 * t + t / 1000000
		dt = (currenttime - previoustime) / 1000000.0
		print "Delta time: d = %f" % dt
		#Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)) :
			#Normalise accelerometer measurement
			recipNorm = (ax * ax + ay * ay + az * az)**-.5
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm
			#Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = q1 * q3 - q0 * q2
			halfvy = q0 * q1 + q2 * q3
			halfvz = q0 * q0 - 0.5 + q3 * q3
			#Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy)
			halfey = (az * halfvx - ax * halfvz)
			halfez = (ax * halfvy - ay * halfvx)
			#Compute and apply integral feedback (if enabled)
			integralFBx += twoKi * halfex * dt;
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx
			gy += integralFBy
			gz += integralFBz
			#Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
		#wwIntegrate rate of change of quaternion
		gx *= (0.5 * dt)
		gy *= (0.5 * dt)
		gz *= (0.5 * dt)
		qa = q0
		qb = q1
		qc = q2
		q0 += (-qb * gx - qc * gy - q3 * gz)
		q1 += (qa * gx + qc * gz - q3 * gy)
		q2 += (qa * gy - qb * gz + q3 * gx)
		q3 += (qa * gz + qb * gy - qc * gx)
		#Normalise quaternion
		recipNorm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)**-1/2 # raiz inversa
		q0 *= recipNorm
		q1 *= recipNorm
		q2 *= recipNorm
		q3 *= recipNorm
		"""
    		rate.sleep() # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop

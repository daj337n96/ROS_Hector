#!/usr/bin/env python

import roslib, rospy, rospkg
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int8
from math import sqrt, cos, sin, pi, atan2, asin
import sys

from hector_constants import *
from ee4308_bringup.msg import EE4308MsgMotion
# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.1

# =============================== SUBSCRIBERS =========================================  
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg
       
def subscribe_turtle_motion(msg):
    global turtle_motion
    turtle_motion = msg

def subscribe_turtle_stop(msg):
    global turtle_stop
    turtle_stop = msg.data
    
def subscribe_sonar(msg):
    global height
    height = msg.range

# ================================ BEGIN ===========================================
def master(sx=2., sy=2., gx=2., gy=2.):
	# ---------------------------------- INITS ----------------------------------------------
	# --- init node ---
	rospy.init_node('hector_motion')

	# --- cache global vars / constants ---
	global msg_motion, turtle_motion, turtle_stop, height
	msg_motion = None
	turtle_motion = None
	turtle_stop = None
	height = None


	# --- Publishers ---
	pub_target = rospy.Publisher('/hector/target', PointStamped, latch=True, queue_size=1)
	msg_target = PointStamped()
	msg_target.header.frame_id = "map"
	msg_target_position = msg_target.point
	msg_target_position.x = sx
	msg_target_position.y = sy
	pub_target.publish(msg_target)

	pub_state = rospy.Publisher('/hector/state', Int8, latch=True, queue_size=1)
	msg_state = Int8()
	msg_state.data = STATE_TAKEOFF
	pub_state.publish(msg_state)

	pub_stop = rospy.Publisher('/hector/stop', Bool, latch=True, queue_size=1)
	msg_stop = Bool()
	msg_stop.data = False
	pub_stop.publish(msg_stop) # ping to others it is ready

	# --- Subscribers ---
	rospy.Subscriber('/hector/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
	rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_turtle_motion, queue_size=1)
	rospy.Subscriber('/turtle/stop', Bool, subscribe_turtle_stop, queue_size=1)
	rospy.Subscriber('/hector/sonar_height', Range, subscribe_sonar, queue_size=1)
	
	state = STATE_TAKEOFF
	check_distance = False
	
	while (height is None or msg_motion is None or turtle_stop is None or \
		turtle_motion is None or rospy.get_time() == 0) and not rospy.is_shutdown():
		pass
	if rospy.is_shutdown():
		return
    
    ######################################################
	t = rospy.get_time()
	while not rospy.is_shutdown():
		if rospy.get_time() > t:
			# --- FSM ---
			print('==================================')
			print('-----Hector Motion-----')
			print('hector motion x= ',msg_motion.x)
			print('hector motion y= ',msg_motion.y)
			print('hector motion z= ',msg_motion.z)
			print('hector motion w= ',msg_motion.w)
			print('hector motion o= ',msg_motion.o)
			print('-----Turtle Motion-----')
			print('turtle motion x= ',turtle_motion.x)
			print('turtle motion y ',turtle_motion.y)
			print('turtle stop value = ',turtle_stop)
			print('-----Current State-----')
			# check if close enough to targets and goals with CLOSE_ENOUGH
			if check_distance:# check if close enough to target
				Di = target_x - msg_motion.x
				Dj = target_y - msg_motion.y
				Dk = target_z - msg_motion.z
				if Di*Di + Dj*Dj + Dk*Dk<= CLOSE_ENOUGH_SQ:# if target reached
					if state == STATE_TAKEOFF:
						state = STATE_TURTLE
					elif turtle_stop == False:
						if state == STATE_TURTLE:
							state = STATE_GOAL
						elif state == STATE_GOAL:
							state = STATE_TURTLE
					elif turtle_stop == True: 
						if state == STATE_GOAL:
							state = STATE_TURTLE
						elif state == STATE_TURTLE:
							state = STATE_BASE
						elif state == STATE_BASE:
							state = STATE_LAND
							
			# use STATE_... constants for states
			if state == STATE_TAKEOFF: # takeoff from hector's base
				print('state = STATE TAKEOFF')
				target_x = sx
				target_y = sy
				target_z = CRUISE_ALTITUDE
				check_distance = True
			elif state == STATE_TURTLE: # fly to turtle
				print('state = STATE_TURTLE')
				target_x = turtle_motion.x
				target_y = turtle_motion.y
				target_z = CRUISE_ALTITUDE
			elif state == STATE_GOAL: # fly to turtle's end goal
				print('state = STATE_GOAL')
				target_x = gx
				target_y = gy
				target_z = CRUISE_ALTITUDE
			elif state == STATE_BASE: # fly to hector's base
				print('state = STATE_BASE')
				target_x = sx
				target_y = sy
				target_z = CRUISE_ALTITUDE
			elif state == STATE_LAND: # land at hector's base
				print('state = STATE_LAND')
				target_x = sx
				target_y = sy
				target_z = 0
				
			# --- Publish state ---
			msg_state.data = state
			pub_state.publish(msg_state)

			# --- Publish target ---
			msg_target_position.x = target_x
			msg_target_position.y = target_y
			msg_target_position.z = target_z
			msg_target.header.seq += 1
			pub_target.publish(msg_target)

			# --- Timing ---
			et = rospy.get_time() - t
			t += ITERATION_PERIOD
			if et > ITERATION_PERIOD:
				print('[HEC MASTER] {} OVERSHOOT'.format(int(et*1000)))
                
    
	# --- Publish stop ---
	msg_stop.data = True
	pub_stop.publish(msg_stop) # ping to others it is ready
	######################################################
    
if __name__ == '__main__':
	try:
		if len(sys.argv) > 1:
			goals = sys.argv[3]
			goals = goals.split('|')
			goals = goals[-1]
			goals = goals.split(',')
			gx = float(goals[0]); gy = float(goals[1])
			master(float(sys.argv[1]), float(sys.argv[2]), gx, gy)
		else:
			master()
	except rospy.ROSInterruptException:
		pass
		
	print('=== [HEC MASTER] Terminated ===')

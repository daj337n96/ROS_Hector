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
	need_traj = False
	state_reach = False
	local_targets_x = 0
	local_targets_y = 0
	
	current_target_x = 0
	current_target_y = 0
	current_target_z = 0
	point_x = sx
	point_y = sy
	point_z = 0.182
	state_number = -1
	
	Dx = 0
	Dy = 0
	Dz = 0
	
	msg_target_position.x = point_x #target_x
	msg_target_position.y = point_y #target_y
	msg_target_position.z = point_z #target_z
	msg_target.header.seq += 1
	pub_target.publish(msg_target)
	
	
	while (height is None or msg_motion is None or turtle_stop is None or \
		turtle_motion is None or rospy.get_time() == 0 ) and not rospy.is_shutdown():
		pass
	if rospy.is_shutdown():
		return
    
    ######################################################
	t = rospy.get_time()
	
	while not rospy.is_shutdown():
		if rospy.get_time() > t:
        # The FSM will always get the most updated position of the Turtle bot. 
        # Once the Hector has moved to a local target/ target(Turtle), the entire FSM would be ran again.
        # This measn that the function state will be eneterd again, allowing the Hector to get the latest Turtle's pose 
        # and running the path through separation & check distance again.
        # This ensures the Hector to accurately arrive directly above the Turtle.
			# --- FSM ---
		 '''print('==================================')
			print('-----Hector Motion-----')
			print('hector motion x= ',msg_motion.x) # the actual motion of Hector
			print('hector motion y= ',msg_motion.y)
			print('hector motion z= ',msg_motion.z)
			print('hector motion w= ',msg_motion.w)
			print('hector motion o= ',msg_motion.o)
			print('-----Turtle Motion-----')
			print('turtle motion x= ',turtle_motion.x) # the actual motion of Turtle
			print('turtle motion y= ',turtle_motion.y)
			print('turtle stop value= ',turtle_stop)
			print('-----Current State-----')'''

			if check_distance: 
                # dist from Hector's pose to Turtle's pose
				Di = target_x - msg_motion.x 
				Dj = target_y - msg_motion.y
				Dk = target_z - msg_motion.z 
                
                # dist from Hector's pose to local targets
				Di_p = point_x - msg_motion.x  
				Dj_p = point_y - msg_motion.y
				Dk_p = point_z - msg_motion.z 
				
				if (Di_p*Di_p + Dj_p*Dj_p + Dk_p*Dk_p) <= CLOSE_ENOUGH_SQ: # check if close enough to local target
					if num_targets < state_number:
						state_reach = True
                    # continue to next local target
					point_x += Dx  
					point_y += Dy
					point_z += Dz
					state_number+=1

				if (Di*Di + Dj*Dj + Dk*Dk <= CLOSE_ENOUGH_SQ) or state_reach: # check if close enough to target (Turtle)
					if state == STATE_TAKEOFF:
						state = STATE_TURTLE
						check_distance = False
					elif turtle_stop != True:
                        # Loops between STATE_TURTLE or STATE_GOAL
						if state == STATE_TURTLE:
							state = STATE_GOAL
						elif state == STATE_GOAL:
							state = STATE_TURTLE
							#turtle_stop = True # activate for shortcut and do one process
						check_distance = False
					elif turtle_stop == True: 
						if state == STATE_GOAL:
							state = STATE_TURTLE
						elif state == STATE_TURTLE:
							state = STATE_BASE
						elif state == STATE_BASE:
							state = STATE_LAND
						elif state == STATE_LAND:
							break # end the program
					state_reach = False
							
				
			if state == STATE_TAKEOFF: # Takeoff from Hector's base
				print('state = STATE TAKEOFF')
				target_x = sx
				target_y = sy
				target_z = CRUISE_ALTITUDE
				check_distance = True
			elif state == STATE_TURTLE: # Fly to Turtle
				print('state = STATE_TURTLE')
				target_x = turtle_motion.x 
				target_y = turtle_motion.y
				target_z = CRUISE_ALTITUDE
				check_distance = True 
			elif state == STATE_GOAL: # Fly to Turtle's end goal
				print('state = STATE_GOAL')
				target_x = gx
				target_y = gy
				target_z = CRUISE_ALTITUDE
				check_distance = True
			elif state == STATE_BASE: # Fly to Hector's base
				print('state = STATE_BASE')
				target_x = sx
				target_y = sy
				target_z = CRUISE_ALTITUDE
				check_distance = True
			elif state == STATE_LAND: # Land at Hector's base
				print('state = STATE_LAND')
				target_x = sx
				target_y = sy
				target_z = 0.182
				check_distance = True
                
			
            # After state transition & when Hector not above turtle, need_traj called
			if((current_target_x-target_x)+(current_target_y-target_y)+(current_target_z-target_z))!=0:
				need_traj = True
                
			
            # Separates the target into local targets before going to check_distance
			if need_traj:
			
				current_target_x = target_x
				current_target_y = target_y
				current_target_z = target_z
				
				if (msg_motion.x!= 0) & (msg_motion.y!=0) & (msg_motion.z!=0):
					Dx = current_target_x - msg_motion.x
					Dy = current_target_y - msg_motion.y
					Dz = current_target_z - msg_motion.z

					num_targets = sqrt(Dx*Dx + Dy*Dy + Dz*Dz) / TARGET_SEPARATION

					Dx /= num_targets*1.0
					Dy /= num_targets*1.0
					Dz /= num_targets*1.0
                    
                    # Separated local targets 
					point_x = msg_motion.x + Dx
					point_y = msg_motion.y + Dy
					point_z = msg_motion.z + Dz
					
				else:
					Dx = current_target_x - sx
					Dy = current_target_y - sy
					Dz = current_target_z - 0.182

					num_targets = sqrt(Dx*Dx + Dy*Dy + Dz*Dz) / TARGET_SEPARATION

					Dx /= num_targets*1.0
					Dy /= num_targets*1.0
					Dz /= num_targets*1.0
					
                    # Separated local targets 
					point_x = sx + Dx
					point_y = sy + Dy
					point_z = 0.182 + Dz
				
				state_number=1
				need_traj = False
				
			# --- Publish state ---
			msg_state.data = state
			pub_state.publish(msg_state)

			# --- Publish separated local target ---
			#print('points = ',point_x,point_y,point_z)
			msg_target_position.x = point_x #target_x
			msg_target_position.y = point_y #target_y
			msg_target_position.z = point_z #target_z
			msg_target.header.seq += 1
			pub_target.publish(msg_target)
			#print("Targets= ",target_x,target_y,target_z)

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

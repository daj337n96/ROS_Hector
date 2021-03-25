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
            # check if close enough to targets and goals with CLOSE_ENOUGH
            ## from project 1   
            if check_distance: # check if close enough to target
                    Di = target_x - msg_motion.x
				Dj = target_y - msg_motion.y
				if Di*Di + Dj*Dj <= CLOSE_ENOUGH_SQ:# if target reached
					target_idx += 1
					if target_idx < num_targets:# still have targets remaining
						previous_x.append(target_x)
						previous_y.append(target_y)
						target_x += Dx
						target_y += Dy
	
						msg_target_position.x = target_x
						msg_target_position.y = target_y
						pub_target.publish(msg_target) # publish new target
					else:
						turnpt_idx -= 1
						if turnpt_idx >= 0:
							update_turnpoint = True 
						else:
							update_goalpoint = True
                ## from project 1    
                    
            # generate targets with TARGET_SEPARATION
            
            # fly to CRUISE_ALTITUDE (you decide value) after takeoff
            # use STATE_... constants for states
        
            # --- Publish state ---
            msg_state.data = STATE_TURTLE
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

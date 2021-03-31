#!/usr/bin/env python

import roslib, rospy, rospkg
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hector_uav_msgs.msg import Altimeter
from std_srvs.srv import Empty# calibrate imu, probably not needed
from math import sqrt, cos, sin, pi, atan2
from numpy import array, transpose, matmul
from numpy.linalg import inv
import sys

from hector_constants import *
from ee4308_bringup.msg import EE4308MsgMotion, EE4308MsgMaster
# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.1

# =============================== SUBSCRIBERS =========================================  
rbt_true = [False, False, False, False]
def subscribe_true(msg):
    tmp = msg.pose.position
    rbt_true[0] = tmp.x
    rbt_true[1] = tmp.y
    rbt_true[2] = tmp.z
    tmp = msg.pose.orientation
    rbt_true[3] = euler_from_quaternion([tmp.x, tmp.y, tmp.z, tmp.w])[2]
    
# ================================ Sensors ===========================================    
rbt_gps = [False, False, False, False]
def subscribe_gps(msg):
    rbt_gps[0] = msg.latitude
    rbt_gps[1] = msg.longitude
    rbt_gps[2] = msg.altitude
    rbt_gps[3] = msg.header.seq

rbt_imu = [False, False, False, False]
def subscribe_imu(msg):
    tmp = msg.linear_acceleration
    rbt_imu[0] = tmp.x
    rbt_imu[1] = tmp.y
    rbt_imu[2] = tmp.z
    rbt_imu[3] = msg.angular_velocity.z
    
rbt_magnet = [False, False, False]
def subscribe_magnetic(msg):
    tmp = msg.vector
    rbt_magnet[0] = tmp.x
    rbt_magnet[1] = tmp.y
    rbt_magnet[2] = msg.header.seq

rbt_alt = [False, False]
def subscribe_altimeter(msg):
    rbt_alt[0] = msg.altitude
    rbt_alt[1] = msg.header.seq

def subscribe_stop(msg):
    global msg_stop
    msg_stop = msg.data
    

# ================================ BEGIN ===========================================
def motion(rx0=2.0, ry0=2.0, rz0 =0.172, ro0=0.0):
    # ---------------------------------- INITS ----------------------------------------------
    # --- init node ---
    rospy.init_node('hector_motion')
    
    # --- cache global vars / constants ---
    global msg_stop
    msg_stop = None
    
    # --- Service: Calibrate ---
    calibrate_imu = rospy.ServiceProxy('/hector/raw_imu/calibrate', Empty)
    calibrate_imu()
    print('[HEC MOTION] Imu calibrated')
    
    # --- Publishers ---
    pub_motion = rospy.Publisher('/hector/motion', EE4308MsgMotion, latch=True, queue_size=1)
    msg_motion = EE4308MsgMotion()
    pub_motion.publish(msg_motion)
    
    pub_pose = rospy.Publisher('/hector/motion_pose', PoseStamped, latch=True, queue_size=1) # for rviz
    msg_pose = PoseStamped()
    msg_pose.header.frame_id = "map"
    msg_pose_position = msg_pose.pose.position
    msg_pose_orientation = msg_pose.pose.orientation
    
    # --- Subscribers ---
    rospy.Subscriber('/hector/ground_truth_to_tf/pose', PoseStamped, subscribe_true, queue_size=1)
    rospy.Subscriber('/hector/fix', NavSatFix, subscribe_gps, queue_size=1)
    rospy.Subscriber('/hector/raw_imu', Imu, subscribe_imu, queue_size=1)
    rospy.Subscriber('/hector/magnetic', Vector3Stamped, subscribe_magnetic, queue_size=1)
    rospy.Subscriber('/hector/altimeter', Altimeter, subscribe_altimeter, queue_size=1)
    rospy.Subscriber('/hector/stop', Bool, subscribe_stop, queue_size=1)
    
    while (not rbt_true[-1] or not rbt_imu[-1] or not rbt_gps[-1] or not rbt_magnet[-1] or\
        rospy.get_time() == 0 or not rbt_alt[-1] or msg_stop is None) and not rospy.is_shutdown():
        pass

    if rospy.is_shutdown():
        return
         
    print('=== [HEC MOTION] Initialised ===')
    
    # ---------------------------------- LOOP ----------------------------------------------
    if USE_GROUND_TRUTH:
        t = rospy.get_time()
        while not rospy.is_shutdown() and not msg_stop:
            if rospy.get_time() > t:
                # publish true motion
                msg_motion.x = rbt_true[0]
                msg_motion.y = rbt_true[1]
                msg_motion.z = rbt_true[2]
                msg_motion.o = rbt_true[3]
                pub_motion.publish(msg_motion)
                
                # publish results to rviz
                msg_pose_position.x = msg_motion.x
                msg_pose_position.y = msg_motion.y
                msg_pose_position.z = msg_motion.z
                tmp = quaternion_from_euler(0, 0, msg_motion.o)
                msg_pose_orientation.x = tmp[0]
                msg_pose_orientation.y = tmp[1]
                msg_pose_orientation.z = tmp[2]
                msg_pose_orientation.w = tmp[3]
                pub_pose.publish(msg_pose)
                
                # iterate
                t += ITERATION_PERIOD
    else:
        ##########################################################
        # --- build sensor functions to calculate conversions ---
        pass
        
        # --- EKF inits --- 
        # EKF S.S Model:  PROCESS = [[0,0], [0,0]]*[[P_k-1], [V_k-1]] + [[0], [0]]*(a + sigma_w)
        #                 MEASUREMENT = [1, 0]*[[P], [V]] + sigma_v 
        
        # Do separate channels for x,y,z,o --> 4 channels in total
        # STATE_X = [[Px], [Vx]], previous_x
        # STATE_Y = [[Py], [Vy]], previous_y
        # STATE_Z = [[Pz], [Vz]], previous_z
        # STATE_O = [[Po], [Vo]], previous_o
        
        # always initialise arrays in two dimensions using [[ . ]]:
        # e.g. X = array([[rx0], [0.]]) # see above for rx0
        # e.g. Px = [[0, 0], [0, 0]]
        pass
        
        # --- Loop EKF ---
        t = rospy.get_time()
        while not rospy.is_shutdown() and not msg_master.stop:
            if rospy.get_time() > t:
                # --- Prediction: from IMU ---
                # x, y, z, o
                # previous_x = take from subscribed rbt_imu[0] 
                # previous_z = rbt_imu[1] 
                # previous_y = rbt_imu[2] 
                # previous_o = rbt_magnet_x/y for position, rbt_imu[3] for speed 
                pass
                
                # --- Correction: incorporate measurements from GPS, Baro (altimeter) and Compass ---
                # separately and when available
                pass
                
                for i in range(10):
                    # --- Publish motion ---
                    msg_motion.x = 0 # m
                    msg_motion.y = 0 # m
                    msg_motion.z = 0 # m
                    msg_motion.o = ((i+1)*36)*DEG2RAD # orientation in z (rad)
                    pub_motion.publish(msg_motion)
                
                # publish results to rviz
                msg_pose_position.x = msg_motion.x
                msg_pose_position.y = msg_motion.y
                msg_pose_position.z = msg_motion.z
                tmp = quaternion_from_euler(0, 0, msg_motion.o)
                msg_pose_orientation.x = tmp[0]
                msg_pose_orientation.y = tmp[1]
                msg_pose_orientation.z = tmp[2]
                msg_pose_orientation.w = tmp[3]
                pub_pose.publish(msg_pose)
                
                print(msg_pose_position.x, msg_pose_position.y, msg_pose_position.z)
                print(msg_pose_orientation.x ,msg_pose_orientation.y ,msg_pose_orientation.z ,msg_pose_orientation.w)
                # --- Iterate ---
                et = rospy.get_time() - t
                t += ITERATION_PERIOD
                if et > ITERATION_PERIOD:
                    print('[HEC MOTION] {}ms OVERSHOOT'.format(int(et*1000)))
                    
                    
        ##########################################################
        
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            motion(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), 0.0)
        else:
            motion()
    except rospy.ROSInterruptException:
        pass
        
    print('=== [HEC MOTION] Terminated ===')

#!/usr/bin/env python

import roslib, rospy, rospkg
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hector_uav_msgs.msg import Altimeter
from std_srvs.srv import Empty# calibrate imu, probably not needed
from math import sqrt, cos, sin, pi, atan, atan2
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
	rbt_imu[0] = tmp.x # accel
	rbt_imu[1] = tmp.y
	rbt_imu[2] = tmp.z
	rbt_imu[3] = msg.angular_velocity.z # gyro
    
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
    
def global2ECEF(latitude, longtitude, altitude): 
	latitude   *= DEG2RAD # convert to rad
	longtitude *= DEG2RAD
	altitude   *= DEG2RAD

	ASquare = RAD_EQUATOR*RAD_EQUATOR
	BSquare = RAD_POLAR*RAD_POLAR
	e_square = 1 - (BSquare/ASquare)
	N = RAD_EQUATOR/(sqrt(1-e_square*sin(latitude)*sin(latitude)))

	Xe = (N + altitude) * cos(latitude) * cos(longtitude)
	Ye = (N + altitude) * cos(latitude) * sin(longtitude)
	Ze = ((BSquare/ASquare)*(N) + altitude)* sin(latitude)

	GPS_array = array([[Xe], [Ye], [Ze]]) # ECEF
	return GPS_array
	
def R_ECEF_to_NED(latitude, longtitude): # 3x3 Rotational matrix for ECEF to NED
	latitude   *= DEG2RAD # convert to rad
	longtitude *= DEG2RAD

	R_e_n = array([[-(sin(latitude)*cos(longtitude)), -sin(longtitude), -(cos(latitude)*cos(longtitude))], 
			[-(sin(latitude)*sin(longtitude)), cos(longtitude),  -(cos(latitude)*sin(longtitude))], 
			[cos(latitude),                   0,           -sin(longtitude)]])

	return transpose(R_e_n)
    
def R_NED_to_MAP(): # 3x3 Rotational matrix for NED to MAP
	R_n_m = array([[cos(0), sin(0),  0], 
		           [sin(0), -cos(0), 0], 
		           [0,        0,    -1]])
		    
	return R_n_m 
	# initial pose in ecef can be taken from --> initial map --> initial NED 
    
def ECEF2MAP(ECEF_initial, MAP_initial):
	ECEF = global2ECEF(rbt_gps[0],rbt_gps[1],rbt_gps[2]) # 3x1
	rotateECEF2NED = R_ECEF_to_NED(rbt_gps[0], rbt_gps[1]) # Get the rotational matrix
	NED_array = matmul(rotateECEF2NED, (ECEF - ECEF_initial)) # Get NED coordinate

	rotateNED2MAP = R_NED_to_MAP() # Get the rotational matrix
	MAP_array = matmul(rotateNED2MAP, NED_array) + MAP_initial # Get MAP coordinate

	return MAP_array 
    
class EKF:
	def __init__(self,x,y,z,o):
		self.x = x  
		self.y = y 
		self.z = z
		self.o = o
	
class SENSOR:
	def __init__(self,x,y,z,o):
		self.x = x  
		self.y = y 
		self.z = z
		self.o = o

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
    
############################## build sensor functions to calculate conversions ##############################

		array11 = array([0])
		array12 = array([[0], [0]])
		array22 = array([[0, 0],[0, 0]])
		
		idenity22 = array([[1, 0],[0, 1]])

        # --- Parameters inits ---
        # Model
		process		= EKF(array12, array12, array12, array12) # 2x1
		measured 	= EKF(0, 0, 0, 0) # 1x1

		predicted 	= EKF(array12, array12, array12, array12) # X(k|k-1) # 2x1
        # Initialize previous states as the Map's initial coordinates 
		previous	= EKF(array([[rx0], [0]]), array([[ry0], [0]]), array([[rz0], [0]]), array([[ro0], [0]])) 
		
		# Covariances initialise as 0
		filtered_covar		 = EKF(idenity22, idenity22, idenity22, array22) # P(k|k) # 2x2
		predicted_covar  	 = EKF(idenity22, idenity22, idenity22, array22) # P(k|k-1) # 2x2
		prev_filtered_covar	 = EKF(idenity22, idenity22, idenity22, array22) # P(k-1|k-1) # 2x2 # Holds the value for predicted_covar

		filtered_state	= previous# X(k|k) # 2x1
		prev_filtered_state	= previous # X(k-1|k-1) # 2x1 

		pass
        # Initial coordinates 
		ECEF_initial = global2ECEF(rbt_gps[0],rbt_gps[1],rbt_gps[2]) # Find the initial pose in ECEF
		MAP_initial = array([[rx0], [ry0], [rz0]]) # Get map initial pose rx0,ry0,rz0
		#print("MAP_initial")
        
        # Initialize sensors's header seq
        GPS_header_seq = rbt_gps[3]
        MAG_header_seq = rbt_magnet[2]
        ALT_header_seq = rbt_alt[1]
        
        # Offset value for Altimeter 
        ALT_offset = rbt_alt[1] 
        
		#################################### Loop EKF ####################################
		t = rospy.get_time()
		previoustime = t

		while not rospy.is_shutdown() and not msg_stop: #msg_master.stop:
			if rospy.get_time() > t:
				current_time = rospy.get_time()
				delta_t = current_time - previoustime
				current_time = previoustime
                
				# --- Noise inits ---
				a_w = EKF(0,0,0,0)
				a_w.x = rbt_imu[0]
				a_w.y = rbt_imu[1]
				a_w.z = rbt_imu[2]
				a_w.o = rbt_imu[3] # noise is from gyro which is angular_vel
                
                # Parameters for constant update within loop
				Gain_K = EKF(array([[0],[0]]), array([[0],[0]]), array([[0],[0]]), array([[0],[0]]))# kf(k)
				sigma_W = array([[0.5*delta_t*delta_t], [delta_t]])
				H_K = array([[1,0]])
				H_K_t = transpose(H_K)
				F_k = array([[1, delta_t], [0, 1]])
				F_k_t = transpose(F_k)
                # Heading requires edited transition model
                sigma_W_o = array([[delta_t], [1]])
				F_k_o = array([[1, 0], [0, 0]]) 
				F_k_o_t = transpose(F_k_o)
	
					
				# --- EKF Measurements for x, y, z, o --- # From GPS & MAG
				gps_robot_frame = ECEF2MAP(ECEF_initial, MAP_initial) # Get GPS in MAP coordinate

				measured.x = matmul([1,0,0],gps_robot_frame) 
				measured.y = matmul([0,1,0],gps_robot_frame)
				measured.z = matmul([0,0,1],gps_robot_frame)
				measured.o = atan2(rbt_magnet[1],rbt_magnet[0])   
				print("=============================================================\n")              
				#print("[Measured]", measured.x,  measured.y,  measured.z,  measured.o)
                

				# --- Prediction: incorporate measurements from IMU (accel & gyro)---
				# --- Equation A ---
						# state prediction X(k|k-1)
				predicted.x = matmul(F_k, prev_filtered_state.x) + sigma_W * a_w.x 
				predicted.y = matmul(F_k, prev_filtered_state.y) + sigma_W * a_w.y
				predicted.z = matmul(F_k, prev_filtered_state.z) + sigma_W * a_w.z 
				predicted.o = matmul(F_k_o, prev_filtered_state.o) + sigma_W_o * a_w.o 
				#print("predicted: ", predicted.x[0], predicted.y[0], predicted.z[0], predicted.o[0])  

				# --- Equation B ---
						# variance = covariance since only 1 param
				predicted_covar.x = matmul((matmul(F_k, prev_filtered_covar.x)), F_k_t) + matmul(sigma_W*IMU_NX, transpose(sigma_W)) 
				predicted_covar.y = matmul((matmul(F_k, prev_filtered_covar.y)), F_k_t) + matmul(sigma_W*IMU_NY, transpose(sigma_W))
				predicted_covar.z = matmul((matmul(F_k, prev_filtered_covar.z)), F_k_t) + matmul(sigma_W*IMU_NZ, transpose(sigma_W))
				predicted_covar.o = matmul((matmul(F_k_o, prev_filtered_covar.o)), F_k_o_t) + matmul(sigma_W_o*IMU_NO, transpose(sigma_W_o))
                

				# --- Correction: incorporate measurements from GPS, Magnetometer, & Altimeter ---
                # Sensor measurements updated based on their header seq
                if(GPS_header_seq != rbt_gps[3]):
                
                    GPS_header_seq = rbt_gps[3] # update seq
                    
                    # --- Equation C ---            
                    Gain_K.x = matmul(matmul(predicted_covar.x, H_K_t), (matmul(matmul(H_K,predicted_covar.x), H_K_t) + GPS_NX)**-1)
                    Gain_K.y = matmul(matmul(predicted_covar.y, H_K_t), (matmul(matmul(H_K,predicted_covar.y), H_K_t) + GPS_NY)**-1)
                    Gain_K.z = matmul(matmul(predicted_covar.z, H_K_t), (matmul(matmul(H_K,predicted_covar.z), H_K_t) + GPS_NZ)**-1)

                    # --- Equation D ---
                    filtered_state.x = predicted.x + matmul(Gain_K.x, (measured.x - matmul(H_K, predicted.x)))
                    filtered_state.y = predicted.y + matmul(Gain_K.y, (measured.y - matmul(H_K, predicted.y)))
                    filtered_state.z = predicted.z + matmul(Gain_K.z, (measured.z - matmul(H_K, predicted.z)))

                    # --- Equation E ---
                    filtered_covar.x = predicted_covar.x - matmul(matmul(Gain_K.x, H_K), predicted_covar.x) 
                    filtered_covar.y = predicted_covar.y - matmul(matmul(Gain_K.y, H_K), predicted_covar.y)
                    filtered_covar.z = predicted_covar.z - matmul(matmul(Gain_K.z, H_K), predicted_covar.z) 
                
                if(MAG_header_seq != rbt_magnet[2]):
                
                    MAG_header_seq = rbt_magnet[2] # update seq
                    
                    # --- Equation C ---  
                    Gain_K.o = matmul(matmul(predicted_covar.o, H_K_t), (matmul(matmul(H_K,predicted_covar.o), H_K_t) + MAG_NO)**-1)                  
                    # --- Equation D ---
                    filtered_state.o = predicted.o + matmul(Gain_K.o, (measured.o - matmul(H_K, predicted.o)))                   
                    # --- Equation E ---
                    filtered_covar.o = predicted_covar.o - matmul(matmul(Gain_K.o, H_K), predicted_covar.o) 
                    
                if(ALT_header_seq != rbt_alt[1]):
                    
                    ALT_header_seq = rbt_alt[1] # update seq
                    
                    alt_reading = rbt_alt[0] - ALT_offset # reading with off set
                    
                    # --- Equation C --- 
                    Gain_K.z = matmul(matmul(predicted_covar.z, H_K_t), (matmul(matmul(H_K,predicted_covar.z), H_K_t) + ALT_NZ)**-1)                   
                    # --- Equation D ---
                    filtered_state.z = predicted.z + matmul(Gain_K.z, (measured.z - matmul(H_K, predicted.z)))
                    # --- Equation E ---
                    filtered_covar.z = predicted_covar.z - matmul(matmul(Gain_K.z, H_K), predicted_covar.z)

                    

				# --- Update ---
						# update the previous parameters for next loop
				previous = predicted 
				prev_filtered_covar = filtered_covar 
				prev_filtered_state = filtered_state 
				#print("Filtered_state: ", filtered_state.x[0], filtered_state.y[0], filtered_state.z[0], filtered_state.o[0])
				
				pass

				# --- Publish motion ---		
				msg_motion.x = filtered_state.x[0]
				msg_motion.y = filtered_state.y[0]
				msg_motion.z = rbt_true[2] # m
				msg_motion.o = -filtered_state.o[0]
				pub_motion.publish(msg_motion)
				print("msg_true: ", str(rbt_true[0]), str(rbt_true[1]), str(rbt_true[2]), str(rbt_true[3]))       
				print("msg_motion: ", str(msg_motion.x), str(msg_motion.y), str(msg_motion.z), str(msg_motion.o))        
				print("\n=============================================================")           

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

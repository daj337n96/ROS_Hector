# ROS_Hector
check state of Hector in FSM before taking the next step
Master controls the STATE and sets the target to go to (pink dot)
    Subscribes to Motion since it overlooks everything 
Motion deals with EKF predict and correction 

###############################_hector_master_############################################
publishes targets

/hector/target
/hector/state
/hector/stop

###############################_hector_motion_############################################
publishes the motion 

/hector/motion
/hector/motion_pose

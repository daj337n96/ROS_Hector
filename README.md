# ROS_Hector
check state of Hector in FSM before taking the next step
Master controls the STATE and sets the target to go to (pink dot)
    Subscribes to Motion since it overlooks everything 
Motion deals with EKF predict and correction 

############################### hector_master ############################################
publishes targets to move to

/hector/target
/hector/state
/hector/stop

############################### hector_motion ############################################
publishes the motion that hector is making

/hector/motion
/hector/motion_pose for RViz

# ROS_Hector
check state of Hector in FSM before taking the next step
Master controls the STATE and sets the target to go to (pink dot)
    Subscribes to Motion since it overlooks everything 
Motion deals with EKF predict and correction 

############################### hector_master ############################################
publishes targets to move to

When GROUND_TRUTH = False the motion published needs to be taken from EKF not set, otherwise the drone would not move
Need to settle EKF first 

/hector/target
/hector/state
/hector/stop

############################### hector_motion ############################################
publishes the motion that hector is making
EKF should comapre taregt with motion to decide actual movement
/hector/motion
/hector/motion_pose for RViz


1/4/21:
create def for Predicted_x, prev_x etc
formula A is to predict the state k based on k-1
formula B is to predict the covariance k based on k-1
use the model as fd and Fk

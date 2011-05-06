#test client for ik_trajectory_tutorial

import roslib
roslib.load_manifest('re_kuka')
import rospy
from re_kuka.srv import ExecuteCartesianIKTrajectory
from geometry_msgs.msg import Pose
import time
import sys
import pdb
import tf
import math

#execute a Cartesian trajectory defined by a root frame, a list of 
#positions (x,y,z), and a list of orientations (quaternions: x,y,z,w)
def call_execute_cartesian_ik_trajectory(frame, positions, orientations):
    rospy.wait_for_service("execute_cartesian_ik_trajectory")

    #fill in the header (don't need seq)
    header = roslib.msg.Header()
    header.frame_id = frame
    header.stamp = rospy.get_rostime()

    #fill in the poses
    poses = []
    for (position, orientation) in zip(positions, orientations):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        poses.append(pose)

    #call the service to execute the trajectory
    print "calling execute_cartesian_ik_trajectory"
    try:
        s = rospy.ServiceProxy("execute_cartesian_ik_trajectory", \
                                   ExecuteCartesianIKTrajectory)
        resp = s(header, poses)
    except rospy.ServiceException, e:
        print "error when calling execute_cartesian_ik_trajectory: %s"%e
        return 0
    return resp.success

#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])

#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    rospy.init_node("test_cartesian_ik_trajectory_executer")
    tf_listener = tf.TransformListener()
    time.sleep(.5) #give the transform listener time to get some frames

    # not needed, fix tutorial
    joint_names = ["arm_1_joint",
                   "arm_2_joint",
                   "arm_3_joint",
                   "arm_4_joint",
                   "arm_5_joint",
                   "arm_6_joint",
                   "arm_7_joint"]

    positions = [[0.7, 0.45, 0.45]]
    orientations = [[0.5 , 0.5 , 0.5 , 0.5]]
    
    #positions = [[0.7, 0.45, 0.45],[0.7, 0.45, 0.32]]
    #orientations = [[0.5 , 0.5 , 0.5 , 0.5],[0.5 , 0.5 , 0.5 , 0.5]]
    
    success = call_execute_cartesian_ik_trajectory("/arm_0_link", positions, orientations)
     
    #R = 0.272
     
    
    #for i in range(0,90):
    #    positions = [[0.7-R*math.sin(i*math.pi/90), 0.45+R*(math.cos(i*math.pi/90)-1), 0.32]]
    #    orientations = [[0.5 , 0.5 , 0.5 , 0.5]]
    #    success = call_execute_cartesian_ik_trajectory("/arm_0_link", positions, orientations)

    #check the final pose
    (trans, rot) = tf_listener.lookupTransform('/arm_0_link', '/arm_7_link', rospy.Time(0))
    print "end Cartesian pose: trans", pplist(trans), "rot", pplist(rot)

    if success:
        print "trajectory succeeded!"
    else:
        print "trajectory failed."

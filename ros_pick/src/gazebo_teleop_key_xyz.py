#!/usr/bin/env python
import time
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import copy
from control_msgs.msg import *
from trajectory_msgs.msg import *
import moveit_commander
import moveit_msgs.msg
# For teleoperation
import sys, select, termios, tty 
import numpy as np

from sensor_msgs.msg import JointState # To receive the current state

reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

# For forward/inverse kinematics
from ur_kinematics import Kinematics
import math

# For Debugging: Should be deleted when committed
reload(sys)  # to enable `setdefaultencoding` again
sys.setdefaultencoding("UTF-8")

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

client = None



def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

# For Teleoperation by Keyboard
msg = """
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Reading from the keyboard  and Publishing to move UR5!
---------------------------
Joint Control: J1 J2 J3 J4 J5 J6
     UP      : q  w  e  r  t  y
     DOWN    : a  s  d  f  g  h     
    (NB: Incremental joint angle = 0.1 (rad))
---------------------------
Pose Command : 1, 2
---------------------------
anything else : stop or CTRL-C to quit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++"""

msg2 = """
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Reading from the keyboard  and Publishing to move UR5!
---------------------------
Position Control:      (NB: Increment = 0.01 (m)) 
    - Forward/Backware/Left/Right (x-y-axis): u / j / h / k
    - Up/Down (z-axis): t / g
Orientation Control:   (NB: Increment = pi/50 (rad))
    - Pitch Up/Down  : w / s
    - Roll Left/Right: a / d
    - Yaw Left/Right : q / e
---------------------------
Pose Command : 1, 2
---------------------------
anything else : stop or CTRL-C to quit
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
=> """


del_q = 0.1
del_x = 0.01
pi = math.pi
del_ori = pi/50

moveBindings = {
		'q':(del_q,0,0,0,0,0),
		'a':(-del_q,0,0,0,0,0),
		'w':(0,del_q,0,0,0,0),
		's':(0,-del_q,0,0,0,0),
		'e':(0,0,del_q,0,0,0),
		'd':(0,0,-del_q,0,0,0),
		'r':(0,0,0,del_q,0,0),
		'f':(0,0,0,-del_q,0,0),
		't':(0,0,0,0,del_q,0),
		'g':(0,0,0,0,-del_q,0),
		'y':(0,0,0,0,0,del_q),
		'h':(0,0,0,0,0,-del_q),
		  }

moveBindings2 = {		
        'u':np.matrix([[0,0,0,del_x],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),
        'j':np.matrix([[0,0,0,-del_x],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),
        'h':np.matrix([[0,0,0,0],[0,0,0,del_x],[0,0,0,0],[0,0,0,0]]),
        'k':np.matrix([[0,0,0,0],[0,0,0,-del_x],[0,0,0,0],[0,0,0,0]]),
        't':np.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,del_x],[0,0,0,0]]),
        'g':np.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,-del_x],[0,0,0,0]]),      
		  }

oriBindings = {
        'w':np.matrix([[math.cos(del_ori),0,math.sin(-del_ori)],[0,1,0],[math.sin(del_ori),0,math.cos(del_ori)]]), # Pitch control: Up
        's':np.matrix([[math.cos(del_ori),0,math.sin(del_ori)],[0,1,0],[math.sin(-del_ori),0,math.cos(del_ori)]]), # Pitch control: Down
        'a':np.matrix([[1,0,0],[0,math.cos(del_ori),math.sin(del_ori)],[0,math.sin(-del_ori),math.cos(del_ori)]]), # Role control: Left
        'd':np.matrix([[1,0,0],[0,math.cos(del_ori),math.sin(-del_ori)],[0,math.sin(del_ori),math.cos(del_ori)]]), # Role control: Right
        'q':np.matrix([[math.cos(del_ori),math.sin(-del_ori),0],[math.sin(del_ori),math.cos(del_ori),0],[0,0,1]]), # Yaw control: Left
        'e':np.matrix([[math.cos(del_ori),math.sin(del_ori),0],[math.sin(-del_ori),math.cos(del_ori),0],[0,0,1]]), # Yaw control: Right
            }

poseBindings={
		'1':(0,0,0,0,0,0),
        '2':(3.14,0,0,0,0,0),
	      }

delBindings={
		'+':(0.01),
        '-':(-0.01),
	      }          

def getKey():
	tty.setraw(sys.stdin.fileno()) 
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1) 
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def ur_get_status():

    current_data = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)
    return current_data

def teleop_key_xyz():
    pose = [0.7,0.22,1.2,0,0,0]
    pose_target = geometry_msgs.msg.Pose()

    # pose = ur_status.actual.positions
    # Initilise UR5 position



    run_flag = 1
    while(run_flag):

        pose_valid_flag = 0 # user input position validity (initialisation) 
        while(pose_valid_flag == 0):
            print(msg)    
            key = getKey()
            print("=> Key input = " + str(key) + "\n")
            if key in moveBindings.keys():
              # Use current joint angles as a guess value for IK
                group_pose_xyz = group.get_current_pose()
                print(group_pose_xyz)
                pose_xyz = [group_pose_xyz.pose.position.x,group_pose_xyz.pose.position.y,group_pose_xyz.pose.position.z,
                            group_pose_xyz.pose.orientation.x,group_pose_xyz.pose.orientation.y,group_pose_xyz.pose.orientation.z]
                print(pose_xyz)

                pose_xyz = np.add(pose_xyz, moveBindings[key])
                #pose_target = pose_xyz # Reconvert the desired xyz onto new pose
                pose_target.position.x = pose_xyz[0]
                pose_target.position.y = pose_xyz[1]
                pose_target.position.z = pose_xyz[2]
                pose_target.orientation.x = pose_xyz[3]
                pose_target.orientation.y = pose_xyz[4]
                pose_target.orientation.z = pose_xyz[5]
       
            else:
                pose_target = pose            
       

            if pose_target is not None:
                pose_valid_flag = 1
            else:
                pose_target = group.get_current_pose()
                print("[!!!Inverse Kinematics Error!!!]: Please move the robot in a different direction")

        if (key == '\x03'):
            break

        group.set_pose_target(pose_target)
        plan1 = group.plan()
        rospy.sleep(0.01)
        try:
            group.go(wait=True)
        except KeyboardInterrupt:
            group.stop()
            raise


def main():
    global group
    try:
        
        rospy.init_node("test_move", anonymous=True, disable_signals=True)        
        # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        teleop_key_xyz()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin)
    main()

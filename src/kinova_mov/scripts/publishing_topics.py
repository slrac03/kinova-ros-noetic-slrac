#!/usr/bin/env python3
import threading
import math

import rospy
import argparse
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped #IK
from sensor_msgs.msg import JointState #DK

#Kortex API
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2

import utilities

TIMEOUT_DURATION = 15

class KinovaControl():

    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
        self.parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
        self.parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
        self.robot_args = self.parser.parse_args()

    def dk_control(self, base):
        pass

    def get_inverse_kinematics(self, position, orientation):
#        with utilities.DeviceConnection.createTcpConnection(self.robot_args) as router:
#            self.base = BaseClient(router)
        self.calc_ik(self.base, position, orientation)

    def calc_ik(self, base, position, orientation):
        try: 
            input_joint_angles = base.GetMeasuredJointAngles()
            pose = base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to get current robot pose")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False
        
        input_ik_data = Base_pb2.IKData()
        input_ik_data.cartesian_pose.x = position.x
        input_ik_data.cartesian_pose.y = position.y
        input_ik_data.cartesian_pose.z = position.z
        input_ik_data.cartesian_pose.theta_x = pose.theta_x 
        input_ik_data.cartesian_pose.theta_y = pose.theta_y
        input_ik_data.cartesian_pose.theta_z = pose.theta_z 

        for joint_angle in input_joint_angles.joint_angles:
            JAngle = input_ik_data.guess.joint_angles.add()
            JAngle.value = joint_angle.value - 1 

        computed_ik_angles = base.ComputeInverseKinematics(input_ik_data)
        computed_ik_angles.joint_angles[2].value += 360.00


KC = KinovaControl()

def getmsg_callback(msg):
    KC.get_inverse_kinematics(msg.pose.position, msg.pose.orientation)
    print("Position {} | Orientation {}".format(msg.pose.position, msg.pose.orientation))
    rospy.loginfo("Inverse Kinematics msg send")

def getdkmsg_callback(msg):
    pass

def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Zero":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def main():

    rospy.init_node("KinovaController")
    rospy.loginfo("Initializing KinovaController node with" + str(KC.robot_args))

    pub = rospy.Publisher("kinova_current", String, queue_size=10)
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.Subscriber("/controller", PoseStamped, getmsg_callback)

    rate = rospy.Rate(10)
    if str(KC.robot_args.ip) == "192.168.1.10":
        with utilities.DeviceConnection.createTcpConnection(KC.robot_args) as router:
            base = BaseClient(router)
            example_move_to_home_position(base)
            while not rospy.is_shutdown():      
                try:
                    joint_angles = base.GetMeasuredJointAngles()
                    pose = base.ComputeForwardKinematics(joint_angles)
                    
                    pub.publish(f"Arm actual pos: x: {pose.x:.3f}, y: {pose.y:.3f}, z: {pose.z:.3f}, "
                        f"theta_x: {pose.theta_x:.2f}, theta_y: {pose.theta_y:.2f}, theta_z: {pose.theta_z:.2f}")

                    js = JointState()
                    js.header = Header()
                    js.header.stamp = rospy.Time.now()
                    #run to make test: roslaunch urdf_tutorial display.launch model:="/home/carlos/catkin_ws/src/kinova_gen3_urdf/urdf/GEN3.urdf" gui:=false
                    #kill the other node
                    js.name = ["joint_1", "joint_2", "joint_3",
                                "joint_4", "joint_5", "joint_6"]
                    js.position = [math.radians(ja.value) for ja in joint_angles.joint_angles]
                    joint_pub.publish(js)
                    #rospy.logwarn("Publishing actual pose")


                except Exception as e:
                    rospy.logerr("Can't get arms angles")

                rate.sleep()  
    else:
        print("Not connected, check ip address")
        rospy.signal_shutdown("Check ip address")

if __name__=="__main__":
    main()
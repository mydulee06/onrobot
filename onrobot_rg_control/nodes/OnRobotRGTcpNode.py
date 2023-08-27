#!/usr/bin/env python3

import math

import rospy
import onrobot_rg_control.baseOnRobotRG
import onrobot_rg_modbus_tcp.comModbusTcp
from std_srvs.srv import Trigger, TriggerResponse
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput
from sensor_msgs.msg import JointState


class OnRobotRGTcp:
    """ OnRobotRGTcp connects to the gripper with Modbus/TCP.

        Attributes:
            gripper (onrobot_rg_control.baseOnRobotRG.onrobotbaseRG):
                instance of onrobotbaseRG used for the connection establishment
            pub (rospy.Publisher): the publisher for OnRobotRGInput

            restartPowerCycle:
                Restarts the power cycle of the gripper.
            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self):
        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = \
            onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = \
            onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        self.joint_names = ["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint", 
                            "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]
        self.mimic_ratios = [1, -1, 1, -1, -1, 1]

        self.L1 = 0.1389215
        self.theta1 = 1.3963
        self.L3 = 0.08
        self.theta3 = 0.93766
        self.dy = -0.0245
        self.dz = 0.055

        # The Gripper status is published on the topic 'OnRobotRGInput'
        self.pub = rospy.Publisher(
            'OnRobotRGInput', OnRobotRGInput, queue_size=1)
        self.joint_states_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=5)

        # The Gripper command is received from the topic 'OnRobotRGOutput'
        rospy.Subscriber('OnRobotRGOutput',
                         OnRobotRGOutput,
                         self.gripper.refreshCommand)

        # The restarting service
        rospy.Service(
            "/onrobot_rg/restart_power",
            Trigger,
            self.restartPowerCycle)

        self.mainLoop()
        
    def widthToJointValue(self, width):
        j_value = math.acos(((width/2) - self.dy - self.L1 * math.cos(self.theta1)) / self.L3) - self.theta3
        return j_value

    def restartPowerCycle(self, request):
        """ Restarts the power cycle of the gripper. """

        rospy.loginfo("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg = []
        while not rospy.is_shutdown():
            # Getting and publish the Gripper status
            status = self.gripper.getStatus()

            self.pub.publish(status)

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            if status.gWDF < 1600:
                j_value = self.widthToJointValue(status.gWDF / 10000)
                msg.position = [j_value * ratio for ratio in self.mimic_ratios]
                self.joint_states_pub.publish(msg)

            rospy.sleep(0.05)
            # Sending the most recent command
            if not int(format(status.gSTA, '016b')[-1]):  # not busy
                if not prev_msg == self.gripper.message:  # find new message
                    rospy.loginfo(rospy.get_name()+": Sending message.")
                    self.gripper.sendCommand()
            prev_msg = self.gripper.message
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        gtype = rospy.get_param('/onrobot/gripper', 'rg6')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node(
            'OnRobotRGTcpNode', anonymous=True, log_level=rospy.DEBUG)
        OnRobotRGTcp()
    except rospy.ROSInterruptException:
        pass

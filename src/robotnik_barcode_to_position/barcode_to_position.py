#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import yaml

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped
from robotnik_msgs.msg import Registers
from nav_msgs.msg import Odometry


class BarcodeToPosition(RComponent):
    """
    This node allows to get the real position of the robot from a barcode scanner.
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.modbus_io_sub_name = rospy.get_param(
            '~modbus_io_sub_name', 'robotnik_modbus_io/registers')
        self.real_pos_yaml_path = rospy.get_param(
            '~real_pos_yaml_path', 'barcode_to_real_positions.yaml')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        self.status_pub = rospy.Publisher(
            '~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher(
            '~status_stamped', StringStamped, queue_size=10)
        self.position_pub = rospy.Publisher(
            '~barcode_scan_position', Odometry, queue_size=10)

        # Subscriber
        self.modbus_io_sub = rospy.Subscriber(
            self.modbus_io_sub_name, Registers, self.modbus_io_sub_cb)
        RComponent.add_topics_health(self, self.modbus_io_sub, topic_id='modbus_io_sub', timeout=1.0, required=True)

        return 0

    def init_state(self):
        self.status = String()
        self.barcode_pos = 0
        self.barcode_pos_updated = False

        self.barcodes = yaml.load(open(self.real_pos_yaml_path))

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        # Publish topic with barcode position
        barcode_real_pos = self.barcode_to_real_pos(self.barcode_pos)
        if (self.barcode_pos_updated == True and barcode_real_pos != None):
            pose_covariance = 0.00000001
            rotation_covariance = 0.00000001
            barcode_pos_msg = Odometry()
            barcode_pos_msg.header.stamp = rospy.Time.now()
            barcode_pos_msg.header.frame_id = "robot_odom"
            barcode_pos_msg.child_frame_id = "robot_base_footprint"
            barcode_pos_msg.pose.pose.position.y = barcode_real_pos
            barcode_pos_msg.pose.covariance[0] = pose_covariance
            barcode_pos_msg.pose.covariance[7] = pose_covariance
            barcode_pos_msg.pose.covariance[14] = pose_covariance
            barcode_pos_msg.pose.covariance[21] = rotation_covariance
            barcode_pos_msg.pose.covariance[28] = rotation_covariance
            barcode_pos_msg.pose.covariance[35] = rotation_covariance

            self.position_pub.publish(barcode_pos_msg)
            self.barcode_pos_updated = False

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def barcode_to_real_pos(self, barcode_pos):
        barcode_pos_meters = barcode_pos / 1000.0
        # Conversion from barcode to real position using the yaml params
        for barcode in self.barcodes["barcodes"]:
            if(barcode_pos_meters >= barcode['barcode_pos'][0] and barcode_pos_meters <= barcode['barcode_pos'][1]):
                return barcode['initial_real_pos'] + barcode_pos_meters
        return None

    def modbus_io_sub_cb(self, msg):
        try:
            self.barcode_pos = msg.registers[0].value
            self.barcode_pos_updated = True
        except:
            rospy.logerr("Error reading barcode position. Is register 0 publishing?")
        self.tick_topics_health('modbus_io_sub')

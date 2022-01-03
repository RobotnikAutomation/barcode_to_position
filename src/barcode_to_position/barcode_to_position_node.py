#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from barcode_to_position import BarcodeToPosition


def main():

    rospy.init_node("barcode_to_position_node")

    rc_node = BarcodeToPosition()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

import rospy
import sys
from threading import Thread, Lock
from std_srvs.srv import Trigger, TriggerResponse

import random

class BakerArmServer:

    def __init__(self, name):

        self.verbose_ = True
        self.name_ = name
        self.displayParameters()
        self.initServices()

        self.mutex_ = Lock()

    def displayParameters(self):
        if not self.verbose_:
            return
        print("========== baker_arm_server Parameters ==========")
        print("todo")

    def initServices(self):
        if self.verbose_:
            print("[BakerArmModule] Initializing services...")
        rospy.Service(self.name_ + '/take_trashcan', Trigger, self.handleTakeTrashcanService)
        rospy.Service(self.name_ + '/empty_trashcan', Trigger, self.handleEmptyTrashcanService)
        rospy.Service(self.name_ + '/rest_position', Trigger, self.handleRestPositionService)
        rospy.Service(self.name_ + '/transport_position', Trigger, self.handleTransportPositionService)
        rospy.Service(self.name_ + '/leave_trashcan', Trigger, self.handleLeaveTrashcanService)

        if self.verbose_:
            print("\t\t... services initialized")

    def handleTakeTrashcanService(self, request):
        # todo rmb-ma
        return TriggerResponse()

    def handleEmptyTrashcanService(self, request):
        # todo rmb-ma
        return TriggerResponse()

    def handleRestPositionService(self, request):
        # todo rmb-ma
        return TriggerResponse()

    def handleTransportPositionService(self, request):
        # todo rmb-ma
        return TriggerResponse()

    def handleLeaveTrashcanService(self, request):
        # todo rmb-ma
        return TriggerResponse()


if __name__ == "__main__":
    try:
        rospy.init_node('baker_arm_module_interface', anonymous=True)
        BakerArmServer('baker_arm_module_interface')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

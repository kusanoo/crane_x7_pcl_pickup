#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.tansformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

class Object_position:

    def __init__(self):
        self.sub = rospy.Subscriber("clusters", MarkerArray, self.callback)
        self.r = rospy.Rate(50)
        self.dx = 0
        self.dy = 0

    def callback(self, msg):
        m = msg.markers[0]
        if (m.ns == "target_cluster"):
            self.dx = m.pose.position.x
            self.dy = m.pose.position.y
 
    def offset(self):
        self.ob_x = self.dx - 0.3
        self.ob_y = self.dy 
    def run(self):
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.5)
                print(self.dx, self.dy)
                self.r.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    rospy.init_node("Object_position")
    Object_position().run()

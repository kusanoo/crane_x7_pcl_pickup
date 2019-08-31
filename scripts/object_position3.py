#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
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
        print("Please put it anywhere within 5 seconds")
        rospy.sleep(5.0)

        if(self.dx == 0 and self.dy == 0):
            print("Wait")

        else:
            self.ob_x = 0.3 - self.dy 
            self.ob_y = -self.dx
            print(self.ob_x, self.ob_y)

    def manipulation(self, x, y):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([1.2, 1.2])
        gripper.go()

        # SRDFに定義されている"vertical"の姿勢にする
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = 0.1
        q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target( target_pose )	# 目標ポーズ設定
        arm.go()				# 実行

        #ハンドを閉じる
        gripper.set_joint_value_target([0.43, 0.43])
        gripper.go()

        # 持ち上げる
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()    

        #移動する
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = 0.2
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()    

        #下ろす
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0.2
        target_pose.position.z = 0.13
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()    

        #ハンドを開く
        gripper.set_joint_value_target([1.2, 1.2])
        gripper.go()

        #少しだけハンドを持ち上げる
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0.2
        target_pose.position.z = 0.2
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()    

        arm.set_named_target("vertical")
        arm.go()

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.offset()
                self.manipulation(self.ob_x, self.ob_y)
                #print("Place it in the next place within 5 seconds")
                #rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    rospy.init_node("Object_position")
    Object_position().run()

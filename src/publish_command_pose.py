#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def publish_command_pose():
    # 初始化 ROS 节点
    rospy.init_node('command_pose_publisher', anonymous=True)
    # 创建发布者，发布到 /command/pose 话题
    pub = rospy.Publisher('/command/pose/hover', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 创建 PoseStamped 消息
        pose_msg = PoseStamped()
        # 设置消息头
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"  # 可根据实际情况修改坐标系

        # 设置位置，x=0, y=0, z=1
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 1.0

        # 设置姿态，无旋转
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # 发布消息
        pub.publish(pose_msg)
        rospy.loginfo("Published hover command at x=0, y=0, z=1")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_command_pose()
    except rospy.ROSInterruptException:
        pass

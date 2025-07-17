#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion

def publish_hover_command():
    rospy.init_node('hover_command_publisher', anonymous=True)
    # 创建发布者，发布到 /supereight/path 话题
    pub = rospy.Publisher('/supereight/path/hover', MultiDOFJointTrajectory, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 创建 MultiDOFJointTrajectory 消息
        trajectory_msg = MultiDOFJointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = "world"
        trajectory_msg.joint_names = ["MAV_body"]

        # 创建第一个悬停点
        hover_point1 = MultiDOFJointTrajectoryPoint()

        # 设置悬停位置，x=0, y=0, z=1
        transform1 = Transform()
        transform1.translation.x = 0.0
        transform1.translation.y = -6.0
        transform1.translation.z = 1.0
        # 设置姿态，无旋转
        transform1.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        hover_point1.transforms = [transform1]

        # 设置速度和加速度为 0
        hover_point1.velocities = []
        hover_point1.accelerations = []

        # 设置时间偏移为 0
        hover_point1.time_from_start = rospy.Duration(0)

        # 创建第二个悬停点
        hover_point2 = MultiDOFJointTrajectoryPoint()

        # 设置悬停位置，x=0, y=0, z=1
        transform2 = Transform()
        transform2.translation.x = 0.0
        transform2.translation.y = 2.0
        transform2.translation.z = 1.0
        # 设置姿态，无旋转
        transform2.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        hover_point2.transforms = [transform2]

        # 设置速度和加速度为 0
        hover_point2.velocities = []
        hover_point2.accelerations = []

        # 设置时间偏移为 1 秒
        hover_point2.time_from_start = rospy.Duration(1)

        # 将悬停点添加到轨迹消息中
        trajectory_msg.points = [hover_point1, hover_point2]

        # 发布消息
        pub.publish(trajectory_msg)
        rospy.loginfo("Published hover command at z = 1m")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_hover_command()
    except rospy.ROSInterruptException:
        pass
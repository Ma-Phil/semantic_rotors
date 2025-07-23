#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion

def publish_hover_command():
    rospy.init_node('hover_command_publisher', anonymous=True)
    # 创建发布者，发布到 /supereight/path 话题
    pub = rospy.Publisher('/supereight/path/hover', MultiDOFJointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz，增加发布频率

    # 等待订阅者连接，确保消息能被接收
    while pub.get_num_connections() < 1:
        if rospy.is_shutdown():
            rospy.logerr("未连接到控制器就关闭了节点")
            return
        rospy.loginfo("等待控制器连接...")
        rate.sleep()

    # 创建 MultiDOFJointTrajectory 消息
    trajectory_msg = MultiDOFJointTrajectory()
    trajectory_msg.header.stamp = rospy.Time.now()
    trajectory_msg.header.frame_id = "world"
    trajectory_msg.joint_names = ["MAV_body"]

    # 创建第一个悬停点
    hover_point1 = MultiDOFJointTrajectoryPoint()

    # 设置悬停位置
    transform1 = Transform()
    transform1.translation.x = 0.0
    transform1.translation.y = -3.8
    transform1.translation.z = 1.0
    # 设置姿态，无旋转
    transform1.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
    hover_point1.transforms = [transform1]

    # 设置速度和加速度为 0
    # 注意：某些控制器可能需要明确设置这些值
    from geometry_msgs.msg import Twist, Accel
    velocity1 = Twist()
    acceleration1 = Accel()
    hover_point1.velocities = [velocity1]
    hover_point1.accelerations = [acceleration1]

    # 设置时间偏移，从开始后2秒到达第一个点
    hover_point1.time_from_start = rospy.Duration(2)

    # 创建第二个悬停点
    hover_point2 = MultiDOFJointTrajectoryPoint()

    # 设置悬停位置
    transform2 = Transform()
    transform2.translation.x = 0.0
    transform2.translation.y = 3.8
    transform2.translation.z = 0.3
    # 设置姿态，无旋转
    transform2.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
    hover_point2.transforms = [transform2]

    # 设置速度和加速度为 0
    velocity2 = Twist()
    acceleration2 = Accel()
    hover_point2.velocities = [velocity2]
    hover_point2.accelerations = [acceleration2]

    # 设置时间偏移，从开始后5秒到达第二个点
    hover_point2.time_from_start = rospy.Duration(5)

    # 将悬停点添加到轨迹消息中
    trajectory_msg.points = [hover_point1, hover_point2]

    # 发布消息（多次发布以确保送达）
    for i in range(5):
        pub.publish(trajectory_msg)
        rate.sleep()
    
    rospy.loginfo("已发布完整轨迹: 先上升到z=1m，然后下降到z=0.3m")
    rospy.loginfo("轨迹执行时间: 2秒到达第一个点，5秒到达第二个点")

if __name__ == '__main__':
    try:
        publish_hover_command()
        # 保持节点运行一小段时间，确保消息发送完成
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
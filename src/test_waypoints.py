#!/usr/bin/env python
import rospy
from mav_msgs.msg import CommandTrajectory
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

def publish_trajectory():
    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/command/trajectory', CommandTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # 定义多个目标点
    waypoints = [
        Point(0, 0, 2),    # 点1：起飞点
        Point(2, 0, 2),    # 点2
        Point(2, 2, 2),    # 点3
        Point(0, 2, 2),    # 点4
        Point(0, 0, 2)     # 点5：回到起点
    ]

    # 每个点的停留时间（秒）
    dwell_time = 5

    while not rospy.is_shutdown():
        for waypoint in waypoints:
            # 创建轨迹消息
            traj = CommandTrajectory()
            traj.header = Header()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = 'world'  # 参考系
            
            # 设置位置
            traj.position = waypoint
            
            # 设置速度（可选）
            traj.velocity.x = 0.0
            traj.velocity.y = 0.0
            traj.velocity.z = 0.0
            
            # 设置加速度（可选）
            traj.acceleration.x = 0.0
            traj.acceleration.y = 0.0
            traj.acceleration.z = 0.0
            
            # 设置偏航角（面向前方）
            traj.yaw = 0.0
            
            # 发布轨迹点
            pub.publish(traj)
            rospy.loginfo(f"Published waypoint: ({waypoint.x}, {waypoint.y}, {waypoint.z})")
            
            # 等待一段时间再发布下一个点
            rospy.sleep(dwell_time)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)
        self.last_ackermann_cmd = AckermannDrive()
        self.obstacles_near = False

    def obstacle_callback(self, msg):
        point_list = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.obstacles_near = self.check_obstacles(point_list)
        self.modify_ackermann_command()
        rospy.loginfo(f"Obstacles near: {self.obstacles_near}")

    def check_obstacles(self, points):
      
        for point in points:
            x, y, z = point
            if x < 1.5 and abs(y) < 1.0:  
                return True
        return False

    def ackermann_callback(self, msg):
        self.last_ackermann_cmd = msg
        rospy.loginfo(f"Received Ackermann command: Speed={msg.speed}, Steering Angle={msg.steering_angle}")
        self.modify_ackermann_command()

    def modify_ackermann_command(self):
        if self.obstacles_near:
            cmd = AckermannDrive()
            cmd.speed = 0.0  
            cmd.steering_angle = 0.0
            rospy.loginfo("Stopping due to an obstacle.")
            self.cmd_pub.publish(cmd)
        else:
            
            rospy.loginfo(f"Publishing original Ackermann command: Speed={self.last_ackermann_cmd.speed}, Steering Angle={self.last_ackermann_cmd.steering_angle}")
            self.cmd_pub.publish(self.last_ackermann_cmd)

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()

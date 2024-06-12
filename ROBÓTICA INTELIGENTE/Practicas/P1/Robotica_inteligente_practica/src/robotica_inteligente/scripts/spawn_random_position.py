#!/usr/bin/env python

import rospy, tf, rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_random_position")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    #Spawn object
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    with open(rospkg.RosPack().get_path("robotica_inteligente")+"/models/object/model.sdf", "r") as f:
        object_model = f.read()

    mu, sigma = np.array([5, 3.5]), 0.25
    point = np.random.normal(mu, sigma)
    while np.linalg.norm(mu-point)>0.5:
        point = np.random.normal(mu, sigma)

    object_pose   =   Pose(Point(x=point[0], y=point[1],    z=0.1),   Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    spawn_model("Object", object_model, "object", object_pose, "world")

    #Spawn camera
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    with open(rospkg.RosPack().get_path("robotica_inteligente")+"/models/realsense/model.urdf", "r") as f:
        object_model = f.read()


    object_pose   =   Pose(Point(x=0, y=0,    z=10.0),   Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    spawn_model("Realsense", object_model, "camera", object_pose, "world")
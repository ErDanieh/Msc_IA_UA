#!/usr/bin/env python
#
# Nodo de ROS para mover el robot UR5 a una posición objetivo mediante MoveIt y 
# abrir y cerrar la pinza mediante el topic /ur5_goal (y los que se consideren opurtunos para coordinar las tareas). 
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf
from tf.transformations import quaternion_from_euler

from math import pi, dist, cos, fabs
from moveit_commander.conversions import pose_to_list

#TODO definir la posición global del robot.
ROBOT_POSITION = geometry_msgs.msg.Point(x=5, y=3, z=0) 
#TODO definir posiciones articulares del brazo (posición home) y la pinza (abierta y cerrada).
#Tener en cuenta que deben estar en radianes.
HOME_JOINT_STATE = [-pi/2, -pi/2, -2*pi/3, -pi/4, pi/2, -pi/4]
CLOSE_JOINT_STATE = [pi/4, pi/4, -pi/32, pi/4, pi/4, -pi/32, pi/4, pi/4, -pi/32]
OPEN_JOINT_STATE = [pi/32, 0, -pi/4, pi/32, 0, -pi/4, pi/32, 0, -pi/4]

class MoveUR5Node(object):

    def __init__(self):
        super(MoveUR5Node, self).__init__()

        ## Inicialización de `moveit_commander`_ y del nodo de ROS:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_UR5_node", anonymous=True)

        ## Inicializar un objeto "RobotCommander". Ofrece información del robot, como su cinemática
        ## y el estado de sus articulaciones.
        self.robot = moveit_commander.RobotCommander()

        ## Inicialización del objeto "PlanningSceneInterface". Ofrece un interfaz remoto para
        ## obtener y especificar la escena en la que trabaja el robot.
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        ## Inicializar el objeto "MoveGroupCommander". Es una interfaz para planificar la 
        ## posición de un grupo de articulaciones. En el UR%, se disponen de tres grupos:
        ## "arm": Las articulaciones del brazo robótico.
        ## "gripper": Las articulaciones de la pinza para abrir y cerrarla.
        ## "gripper_mode": La pinza tiene distintos modos de agarre, que permite mover los dedos en otras direcciones.
        ## En principio, este grupo no es necesario utilizarlo en esta práctica.

        ## La interfaz se utiliza para planificar y ejecutar los movimientos:
        self.move_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Variables
        self.box_name = ""
        self.object_position=geometry_msgs.msg.Point()

        #Eliminar todos los objetos de la escena si los hay.
        self.scene.remove_attached_object(self.move_arm.get_end_effector_link())
        self.scene.remove_world_object()
        #Ir a la posición home
        self.go_to_joint_gripper_state(OPEN_JOINT_STATE)
        self.go_to_joint_arm_state(HOME_JOINT_STATE)

        #TODO considerar más publishers y subscribers para la comunicación entre ambos robots.

        ## Publishers definition

        # Subscribers definition
        self.position_subscriber = rospy.Subscriber("/pose_array",
            geometry_msgs.msg.PoseArray, self.detection_callback, queue_size=1)
        
    def detection_callback(self, pose_array:geometry_msgs.msg.PoseArray):
        # Comprobar si se ha detectado el objeto
        # Esto se comunica mediante la posición en z.
        object_position=pose_array.poses[0].position
        if object_position.z<0: return

        self.object_position=object_position
    

    def go_to_joint_arm_state(self, joint_goal):
        ## Movimiento a una posición articular del brazo
        ## El orden de las articulaciones es el siguiente: shoulder_pan_joint, shoulder_lift_join,
        ## elbow_joint, wrist1_joint, wrist2_joint, wrist3:joint
        print("Posición articular objetivo del brazo robótico:")
        print(joint_goal)

        if len(joint_goal)!=6:
            print("Posición articular no válida, el UR5 tiene 6 articulaciones.")
            return False

        # El comando go() puede ser utilizado con valores articulares, poses o
        # sin parámetros si ya se ha establecido un objetivo para el grupo.
        self.move_arm.go(joint_goal, wait=True)

        # La función stop() se asegura de que no hay movimiento residual.
        self.move_arm.stop()

        # Comprobar que ha llegado al objetivo
        current_joints = self.move_arm.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.015)
    
    def go_to_joint_gripper_state(self, joint_goal):
        ## Movimiento a una posición articular de la pinza
        ## La pinza tiene 9 articulaciones, tres para cada dedo.
        print("Posición articular objetivo de la pinza:")
        print(joint_goal)

        if len(joint_goal)!=9:
            print("Posición articular no válida, la pinza tiene 9 articulaciones.")
            return False

        # El comando go() puede ser utilizado con valores articulares, poses o
        # sin parámetros si ya se ha establecido un objetivo para el grupo.
        self.move_gripper.go(joint_goal, wait=True)

        # La función stop() se asegura de que no hay movimiento residual.
        self.move_gripper.stop()

        # No comprobamos si lo ha conseguid pues abrir y cerrar la garra solo va a
        # fallar cuando no lo cierra completamente al agarrar el objeto, y en ese
        # caso cumple su objetivo también.

    def go_to_pose_arm_goal(self, pose_goal:geometry_msgs.msg.Pose):
        ## Movimiento a una pose global, definida por la posición cartesiana y orientación objtivo
        ## de la muñeca del UR5.
        print("Posición cartesiana objetivo:")
        print(pose_goal)
        # Establecemos la posición actual y mediante el comando go() planificamos y
        # ejecutamos el movimiento. Nos devuelve si ha podido realizarlo.
        self.move_arm.set_pose_target(pose_goal)
        success = self.move_arm.go(wait=True)

        # La función stop() se asegura de que no hay movimiento residual.
        self.move_arm.stop()

        # Siempre es bueno limpiar las poses objetivo.
        self.move_arm.clear_pose_targets()

        # Comprobar que ha llegado al objetivo
        current_pose = self.move_arm.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.1)

    def add_object(self, object_position:geometry_msgs.msg.Point):
        ## Añadir el objeto a la escena. La posición del objeto (x, y)
        ## debe ser respecto a la base del robot.
        print("Añadiendo el objeto a la posición x={}, y={}",object_position.x, object_position.y)
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = "world"
        object_pose.pose.position = object_position
        object_pose.pose.orientation.w = 1.0
        object_pose.pose.position.z = 0.1  
        self.box_name = "object"
        self.scene.add_box(self.box_name, object_pose, size=(0.1, 0.1, 0.2))


    def attach_object(self):
        ## Acoplar el objeto con la pinza. Manipular objetos requiere que el robot pueda tocarlos sin que el
        ## planificador considere el contacto como una colisión. 
        touch_links = self.robot.get_link_names(group="gripper")
        self.scene.attach_box(self.move_arm.get_end_effector_link(), self.box_name, touch_links=touch_links)


    def detach_object(self):
        ## Desacoplar el objeto.
        self.scene.remove_attached_object(self.move_arm.get_end_effector_link(), name=self.box_name)

    def remove_object(self):
        ## Remover el objeto, primero debe ser desacoplado.
        self.scene.remove_world_object(self.box_name)


    #TODO definir funciones para realizar la tarea de agarrar el objeto y dejarlo en el robot. Considerar una máquina de estados para coordianr el proceso.
    #Nota: Se puede obtener la pose actual del brazo (posición y orientación) mediante "self.move_arm.get_current_pose()"
    
    def run(self):
        #Bucle de control
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self.object_position is None:
                continue

            print(self.object_position)

            self.go_to_joint_gripper_state(OPEN_JOINT_STATE)

            self.goal_pose = geometry_msgs.msg.Pose()

            self.goal_pose.position.x = self.object_position.x - ROBOT_POSITION.x
            self.goal_pose.position.y = self.object_position.y - ROBOT_POSITION.y
            self.goal_pose.position.z = 0.7

            self.goal_pose.orientation = self.move_arm.get_current_pose().pose.orientation

            self.go_to_pose_arm_goal(self.goal_pose)

            self.goal_pose.position.z = 0.4

            self.go_to_pose_arm_goal(self.goal_pose)

            self.go_to_joint_gripper_state(CLOSE_JOINT_STATE)

            self.attach_object()

            self.go_to_joint_arm_state(HOME_JOINT_STATE)
            
            rate.sleep()

    
def all_close(goal, actual, tolerance):
    """
    Un método para comprobar que los valores de dos listas se encuientran dentro de un margen de tolerancia.
    Para una pose, se compara también el ángulo entre los dos cuaterniones.
    @param: goal       A list of floats, a Pose or a PoseStamped or Point
    @param: actual     A list of floats, a Pose or a PoseStamped or Point
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
    
    elif type(goal) is geometry_msgs.msg.Point:
        x0, y0, z0 = actual.x, actual.y, actual.z
        x1, y1, z1 = goal.x, goal.y, goal.z
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        return d <= tolerance

    return True


def main():
    try:
        print("Init move_UR5_node")
        node = MoveUR5Node()
        node.run()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    
    
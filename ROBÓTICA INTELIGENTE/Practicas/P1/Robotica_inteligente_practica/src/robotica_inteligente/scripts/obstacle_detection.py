#!/usr/bin/env python

# Este nodo desarrollado en python permite la deteccion de obstaculos en el escenario mediante la nube de puntos de un sensor LiDAR. 
# Mediante las coordenadas XYZ de cada punto de la nube, se pueden filtrar los obstaculos que superen una altura Z en el escenario

# Se tiene una plantilla que facilita el desarrollo de este nodo y se muestran con "TODO" donde se deben agragar las funciones necesarias

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

# Declaracion de los publishers
pub_obstacles = None
pub_freezone  = None

# Declaracion de la altura y radio
# Estas variables globales se define el radio donde se detectan los obstaculos
# y la altura desde que se van a considerar obstaculos. Ambas variables estan en unidades de metros.
#TODO 
# Colocar los valores adecuados de altura y radio para detectar los obstaculos que rodean al robot.
# Importante: Considerar que el sensor LiDAR esta sobre el robot. Los puntos de la nube de puntos bajo el sensor 
# tienen un valor negativo

altura = -0.8
radio = 5

def filter_obstacles_function(point_cloud_in, altura):

    # Funcion que tiene como entrada un mensaje de ROS tipo PointCloud2 y entrega una nube de puntos
    # la cual contiene los obstaculos detectados segun una altura dada.

    # Conversion del mensaje PointCloud2 a un arreglo con los valores de x,y,z
    pc_data = pc2.read_points(point_cloud_in, field_names=("x", "y", "z"), skip_nans=True)

    # Arreglo con los parametros de los obstaculos detectados
    # inicializado de la variable como vacio.
    obstacles_points = []

    # Lazo for para añadir los puntos correspondientes a obstaculos detectados
    # El criterio de deteccion es dependiendo de la altura del objeto, al ser el objeto mayor a una altura dada
    # es considerado como posible obstaculo
    for point in pc_data:

        x, y, z = point

        #TODO 
        # Dada las variables x,y,z añadir al arreglo obstacles_points los puntos que sean mayores a la altura
        # Importante: Los puntos añadidos a obstacle_points se proyectan al valor de altura, es decir que.
        # las coordenadas de un obstaculo (x,y,z) se cambiaran a (x,y,altura)

        if altura < z:   # condicion de altura comparada con Z
            
            # Agregar punto a los obstaculos detectados:
            obstacles_points.append([x, y, altura])

    return obstacles_points

def free_zone_function(point_cloud_in, radio,altura):

    # Funcion que tiene como entrada un mensaje de ROS tipo PointCloud2 y entrega una nube de puntos 
    # con un radio y altura dada. Esta nube de puntos de salida representa las zonas libres de obstaculos 

    # Conversion del mensaje PointCloud2 a un arreglo con los valores de x,y,z
    pc_data = pc2.read_points(point_cloud_in, field_names=("x", "y", "z"), skip_nans=True)
    
    # Arreglo con los parametros de la nube de puntos de zona libre de obstaculos
    free_zone = []

    # Lazo for para añadir los puntos correspondientes a zonas libres de obstaculos
    # El criterio de deteccion es dependiendo de un radio, si no se detectan objetos en el radio se genera la nube de puntos
    # que corresponde a la zona libre

    for point in pc_data:
        x, y, z = point
        
        #TODO
        # Verificar si el punto esta fuera del radio dada la distancia x,y 
        if np.sqrt(x**2 + y**2) > radio:

            # Para crear un radio de zonas libres se debe conocer el angulo de cada punto segun su coordenada x,y
        
            # Calcular el angulo segun de las coordenadas XY (arcotangte)
            ang = np.arctan2(y,x)

            # Calcular las nuevas coordenadas x, y segun el angulo y el radio 
            new_x = radio*np.cos(ang)
            new_y = radio*np.sin(ang)
            
            # Agregar punto al anillo con componente z igual a altura
            free_zone.append([new_x, new_y, altura])


    return free_zone

def publish_topics(obstacles_points,free_zone_points):

    # Funcion para publicar los topics de obstaculos y de zonas libres

    # Declaracion de variables globales
    global pub_obstacles, pub_freezone

    # Encabezado de los topics a publicar, ambos topics deben ir en el frame del sensor Velodyne
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "blue/velodyne"

    # Mensajes para publicar los obstaculos y la zona libre
    obstacles_msg = pc2.create_cloud_xyz32(header, obstacles_points)
    free_zone_msg = pc2.create_cloud_xyz32(header, free_zone_points)

    # Mensajes enviados a los publishers
    pub_obstacles.publish(obstacles_msg)
    pub_freezone.publish(free_zone_msg)
    
def point_cloud_callback(msg):

    # Funcion callback que tiene como entrada el mensaje de la nube de puntos tipo PointCloud2
    # Este mensaje es enviado a las funciones de deteccion de obstaculos y deteccion de zonas libres de obstaculos

    # Declaracion de variables globales
    global altura, radio    

    # Nube de puntos de los objetos detectados generado por la funcion "filter_obstacles_function"
    obstacles_points = filter_obstacles_function(msg, altura)

    # Nube de puntos de las zonas libres generado por la funcion "free_zone_function"
    free_zone_points = free_zone_function(msg, radio,altura)

    # Publica los obstaculos y la zona libre
    publish_topics(obstacles_points, free_zone_points)

def main():
    rospy.init_node('point_cloud_filter_node', anonymous=True)

    global pub_obstacles, pub_freezone

    point_cloud_topic = "/blue/velodyne_points"

    # Topico para publicar obstaculos
    pub_obstacles = rospy.Publisher("/obstacles", PointCloud2, queue_size=10)
    pub_freezone  = rospy.Publisher("/free_zone", PointCloud2, queue_size=10)

    # Funcion de suscripcion de la nube de puntos para filtrar y publicar los posibles obstaculos
    rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

    # Lazo para mantener el nodo activo
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
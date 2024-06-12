#!/usr/bin/env python

# Este nodo desarrollado en python permite la localizacion de objetos (coordenadas xyz) que visualiza una camara,
# esto con la ayuda de una camara RGB y una imagen de profundidad.
# Mediante el fitrado por color de una imagen RGB a HSV, se obtiene el centroide de cada objeto detectado y,
# conociendo la profundidad (Distancia camara-objeto) y los parametros intrinsecos de la camara,
# se logra calcular las coordendas XYZ de los objetos con respecto a la camara.

# Se tiene una plantilla que facilita el desarrollo de este nodo y se muestran con "TODO" donde se deben agragar las funciones necesarias


import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from math import sqrt, pow

# Declarar variables globales ( publishers y bandera de objetos detectados)
filtered_obj_pub = None
filtered_blue_pub = None
pose_array_pub = None
obt_detec = False
robot_detec = False


def filter_img_objects(color_image, lower, upper):
    # Funcion para filtrar una imagen RGB a un color dado en rango HSV
    # El resultado es la imagen filtrada, el centriode del objeto en coordeandas de la image
    # y si hubo o no deteccion del objeto.

    # Convertir la imagen de RGB a HSV para filtrado
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Filtrar la imagen y obtener mask
    mask = cv2.inRange(hsv_image, lower, upper)
    filtered_image = cv2.bitwise_and(color_image, color_image, mask=mask)

    # Busca los contornos de mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # En estas variables se deben guardar el cetroide del objeto en coordenadas de pixeles,
    # y una bandera por si se detecta o no el objeto.

    centroid = None  # (px,py)
    detection = True  # bandera para saber si existe o no objeto detectado

    # Calcular el centroide de mask
    # Dado ya los contornos del objeto filtrado, se puede buscar el contorno con mayor area (cv.contourArea),
    # y se puede calcular el centroide del objeto (px,py) mediante los momentos de la imagen (cv2.moments)
    if contours:
        # Encuentra el contorno de mayor area para concer su centroide
        max_contour = max(contours, key=cv2.contourArea)

        # Calculo los momentos del contorno
        M = cv2.moments(max_contour)

        # Calculo las coordenadas del centroide
        px = int(M["m10"] / M["m00"])
        py = int(M["m01"] / M["m00"])

        centroid = (px, py)
    else:
        # bandera para decir que no existio objeto en la mascara
        detection = False

    return filtered_image, centroid, detection


def img_xyz(centroid, depth, camera_matrix):

    # Funcion que convierte un pixel dado por (centroid) de una imagen de profundidad
    # en una coordenada XYZ. Esto se logra con la ayuda de los parametros intrinsecos de la camara (camera_matrix)
    # El resultado es los valores x,y,z del objeto detectado

    # Se debe extraer los parametros intrinsecos de la matriz de calibracion (mensaje camera_matrix) dado
    # el orden mostrado:

    # Orden de los parametros intrinsecos de la matriz de calibracion
    # fx 0  cx
    # 0  fy cy
    # 0  0  1

    fx = camera_matrix[0]
    cx = camera_matrix[2]
    fy = camera_matrix[4]
    cy = camera_matrix[5]

    # Dado los parametros intrinsecos  de la camara (fx,fy,cx,cy) y el centroide del objeto (px,py)
    # calcular las coordeandas x,y,z del objeto.

    x = (centroid[0] - cx) * depth / fx
    y = (centroid[1] - cy) * depth / fy
    z = depth / 1000

    return x, y, z


def color_image_callback(color_image_msg):

    # Funcion callback de la imagen RGB.
    # En este callback al tener un mensaje de entrada de la iamgen RGB, se filtra la imagen y se publica
    # dos mensajes con las imagenes filtradas de los objetos detectados (objeto rojo y robot Blue)

    # definicion de variables globales
    global filtered_obj_pub, filtered_blue_pub, centroide_obj, centroide_blue, obt_detec, robot_detec

    # Convertir el mensaje de imagen a una imagen OpenCV
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")

    # Filtrar robot Blue
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([120, 255, 255])
    filtered_blue, centroide_blue, robot_detec = filter_img_objects(
        color_image, lower_blue, upper_blue
    )

    # Publicar la imagen filtrada del robot
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_blue, encoding="bgr8")
    filtered_blue_pub.publish(filtered_image_msg)

    # Filtrar objeto rojo (limites de la mascara en HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    filtered_obj, centroide_obj, obt_detec = filter_img_objects(
        color_image, lower_red, upper_red
    )

    # Publicar la imagen filtrada del objeto
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_obj, encoding="bgr8")
    filtered_obj_pub.publish(filtered_image_msg)


def depth_image_callback(depth_image_msg):

    # Funcion callback de la imagen de profundidad
    # En este callback al tener un mensaje de entrada de la imagen de profundidad, se obtiene el dato
    # profundidad de un pixel dado el centroide(px,py). Si no existe el centroide por que no
    # se detecto un objeto, la profundidad no se calcula.

    # Decalracion de variables globales
    global centroide_obj, centroide_blue, depth_obj, depth_blue, obt_detec, robot_detec

    # Convertir el mensaje de imagen de profundidad a una imagen OpenCV
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg)

    # Selecciona los datos de la imagen de profundidad dado un centroide
    # Si la variable de objeto o robot detectado es False no se tiene profunidad.

    # Dada la descripcion anterior, calcular la distancia Camara-objeto (profundidad)
    # del objeto y el robot en el escenario.
    # Se puede acceder a los datos
    # Importante: Los datos de la imagen de profundidad estan en milimetros, se debe convertir el dato a metros.

    if obt_detec:
        depth_obj = (
            depth_image[centroide_obj[1], centroide_obj[0]] / 1000
        )  # Convert depth from millimeters to meters

    if robot_detec:
        depth_blue = (
            depth_image[centroide_blue[1], centroide_blue[0]] / 1000
        )  # Convert depth from millimeters to meters


def camera_info_callback(camera_info_msg):

    # Funcion callback de la informacion de la camara.
    # En este callback se obtiene los parametros intrinsecos de la camara.
    # Despues de tener los datos de imagen RGB y la imagen de profundidad,
    # se localiza el objeto con respecto a la camara.

    # Decalracion de variables globales
    global centroide_obj, centroide_blue, depth_obj, depth_blue, pose_array_pub, obt_detec, robot_detec

    # Calculos para la localizacion del objeto
    # Se inicializa la variables XYZ de cada objeto.
    # Si no se detecta un objeto en la imagen, se coloca -1 en z y en x,y 0,0.

    x_obj, y_obj, z_obj = 0, 0, -1
    x_blue, y_blue, z_blue = 0, 0, -1

    if obt_detec:
        x_obj, y_obj, z_obj = img_xyz(centroide_obj, depth_obj, camera_info_msg.K)

    if robot_detec:
        x_blue, y_blue, z_blue = img_xyz(centroide_blue, depth_blue, camera_info_msg.K)

    # Invertimos el eje Y para que coincida con los ejes globales.
    y_obj, y_blue = -y_obj, -y_blue

    # Imprime en pantalla los resultados al localizar los objetos
    print("XYZ object: {:.3f}, {:.3f}, {:.3f}".format(x_obj, y_obj, z_obj))
    print("XYZ robot : {:.3f}, {:.3f}, {:.3f}".format(x_blue, y_blue, z_blue))

    # Crear un mensaje PoseArray
    pose_array_msg = PoseArray()

    # Configurar el encabezado del mensaje (timestamp y marco de referencia)
    pose_array_msg.header.stamp = rospy.Time.now()
    pose_array_msg.header.frame_id = "camera_link"

    # Pose del objeto
    pose_obj = Pose()
    pose_obj.position.x = x_obj
    pose_obj.position.y = y_obj
    pose_obj.position.z = z_obj
    pose_obj.orientation.w = 1.0

    # Pose del robot
    pose_blue = Pose()
    pose_blue.position.x = x_blue
    pose_blue.position.y = y_blue
    pose_blue.position.z = z_blue
    pose_blue.orientation.w = 1.0

    # Agregar la pose del objeto al PoseArray
    pose_array_msg.poses.append(pose_obj)
    # Agregar la pose del robot al PoseArray
    pose_array_msg.poses.append(pose_blue)

    # Publicar el PoseArray
    pose_array_pub.publish(pose_array_msg)


def ros_node():
    rospy.init_node("Object_localization", anonymous=True)

    # Subscriber de los topics de:
    # - La imagen RGB e la camara
    # - La imagen de profundidad de la camara
    # - La informacion de la camara donde estan los parametros intrinsecos de la camara  (fx,fy,cx,cy)

    # Agregar los topics a los que se debe suscribir el programa. Se tiene como ejemplo como suscribirse
    # Al topic de la imagen RGB de la camara "/camera/color/image_raw".
    # Se debe completar los topics de la imagen de profundidad y de la inforamcion de la camara.

    color_image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, color_image_callback
    )
    depth_image_sub = rospy.Subscriber(
        "/camera/depth/image_raw", Image, depth_image_callback
    )
    camera_info_sub = rospy.Subscriber(
        "/camera/depth/camera_info", CameraInfo, camera_info_callback
    )

    # variables globales
    global filtered_obj_pub, filtered_blue_pub, pose_array_pub

    # Publisher resultados ( imagenes filtradas y poses de los objetos)
    filtered_obj_pub = rospy.Publisher("/filtered_image/object", Image, queue_size=1)
    filtered_blue_pub = rospy.Publisher("/filtered_image/robot", Image, queue_size=1)
    pose_array_pub = rospy.Publisher("/pose_array", PoseArray, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    try:
        ros_node()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
import sys
import os
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
#from ejercicios_flexion import calcular_trayectoria
# Añadir la carpeta scripts al path
script_dir = os.path.join(os.path.dirname(__file__), '../scripts')
sys.path.append(script_dir)
from ejercicios_flexion import calcular_trayectoria

def main():
    rospy.init_node('trajectory_publisher_node')
    pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    
    # Definir la posición inicial
    hombro_ini = [0, 10, 0]
    codo_ini = [0, 0, 0]
    muneca_ini = [0, -8, 0]
    
    # Definir el ángulo de flexión
    grados_flexion_muneca = 100

    # Calcular la trayectoria
    trayectoria_muneca = calcular_trayectoria(hombro_ini, codo_ini, muneca_ini, grados_flexion_muneca)
    
    if trayectoria_muneca is None:
        rospy.logerr("La posición inicial no es correcta.")
        return

    # Crear el mensaje de trayectoria
    path_msg = Path()
    path_msg.header.frame_id = "world"
    path_msg.header.stamp = rospy.Time.now()

    for punto in trayectoria_muneca:
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(punto[0], punto[1], punto[2])
        path_msg.poses.append(pose)

    rate = rospy.Rate(1)  # Publicar la trayectoria a 1 Hz
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()  # Actualizar el tiempo
        pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

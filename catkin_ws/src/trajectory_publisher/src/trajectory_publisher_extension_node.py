#!/usr/bin/env python

import rospy
import sys
import os
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import math
#from ejercicios_flexion import calcular_trayectoria
# Añadir la carpeta scripts al path
# script_dir = os.path.join(os.path.dirname(__file__), '../scripts')
# sys.path.append(script_dir)
# from ejercicios_flexion import calcular_trayectoria

def verificar_posicion_inicial(hombro_ini, codo_ini, muneca_ini,longitud_brazo,longitud_antebrazo):
    # Verifica que la posición inicial sea correcta
    if hombro_ini[0] != 0 or hombro_ini[1] != longitud_brazo or hombro_ini[2] != 0:
        return False
    if codo_ini[0] != 0 or codo_ini[1] != 0 or codo_ini[2] != 0:
        return False
    if muneca_ini[0] != longitud_antebrazo or muneca_ini[1] != 0 or muneca_ini[2] != 0:
        return False
    return True

def calcular_trayectoria(hombro_ini, codo_ini, muneca_ini, grados_muneca,longitud_brazo,longitud_antebrazo):
    # Verifica la posición inicial
    if not verificar_posicion_inicial(hombro_ini, codo_ini, muneca_ini,longitud_brazo,longitud_antebrazo):
        print("La posición inicial no es correcta.")
        return None

    # Calcula la trayectoria de la muñeca
    trayectoria_muneca = []
    for t in range(grados_muneca + 1):
        angulo_muneca = math.radians(t)
        # La muñeca se mueve siguiendo un arco en el plano XY
        muneca = [
            round(longitud_antebrazo * math.cos(angulo_muneca),4),
            round(-longitud_antebrazo * math.sin(angulo_muneca),4),
            muneca_ini[2] 
        ]
        trayectoria_muneca.append(muneca)

    return trayectoria_muneca

def main():
    rospy.init_node('trajectory_publisher_node')
    pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    
    # Obtener parámetros del archivo de lanzamiento 
    longitud_brazo = rospy.get_param('~longitud_brazo')
    longitud_antebrazo = rospy.get_param('~longitud_antebrazo')
    hombro_ini = [0,longitud_brazo,0]
    codo_ini = [0,0,0]
    muneca_ini = [longitud_antebrazo,0,0]
    grados_muneca = rospy.get_param('~grados_muneca')

   
    
    rospy.loginfo(f"longitud_brazo: {longitud_brazo} (type: {type(longitud_brazo)})")
    rospy.loginfo(f"longitud_antebrazo: {longitud_antebrazo} (type: {type(longitud_antebrazo)})")
    rospy.loginfo(f"hombro_ini: {hombro_ini}  (type: {type(hombro_ini)})") 
    rospy.loginfo(f"codo_ini: {codo_ini}") 
    rospy.loginfo(f"muneca_ini: {muneca_ini}") 
    rospy.loginfo(f"grados_muneca: {grados_muneca} (type: {type(grados_muneca)})")


    # Calcular la trayectoria
    trayectoria_muneca = calcular_trayectoria(hombro_ini, codo_ini, muneca_ini, grados_muneca,longitud_brazo,longitud_antebrazo)
    
    if trayectoria_muneca is None:
        rospy.logerr("La trayectoria no se puede obtener.")
        return
    if trayectoria_muneca:
    	for punto in trayectoria_muneca:
            print(punto)
    

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

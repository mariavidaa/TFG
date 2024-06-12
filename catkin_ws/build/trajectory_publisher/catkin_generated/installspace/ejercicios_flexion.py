import math

# Longitudes de los segmentos del brazo
longitud_brazo = 10
longitud_antebrazo = 8

def verificar_posicion_inicial(hombro_ini, codo_ini, muneca_ini):
    # Verifica que la posición inicial sea correcta
    if hombro_ini[0] != 0 or hombro_ini[1] != longitud_brazo or hombro_ini[2] != 0:
        return False
    if codo_ini[0] != 0 or codo_ini[1] != 0 or codo_ini[2] != 0:
        return False
    if muneca_ini[0] != 0 or muneca_ini[1] != -longitud_antebrazo or muneca_ini[2] != 0:
        return False
    return True

def calcular_trayectoria(hombro_ini, codo_ini, muneca_ini, grados_flexion_muneca):
    # Verifica la posición inicial
    if not verificar_posicion_inicial(hombro_ini, codo_ini, muneca_ini):
        print("La posición inicial no es correcta.")
        return None

    # Calcula la trayectoria de la muñeca
    trayectoria_muneca = []
    for t in range(grados_flexion_muneca + 1):
        angulo_muneca = math.radians(t)
        # La muñeca se mueve siguiendo un arco en el plano XY
        muneca = [
            longitud_antebrazo * math.sin(angulo_muneca),
            -longitud_antebrazo * math.cos(angulo_muneca),
            muneca_ini[2] 
        ]
        trayectoria_muneca.append(muneca)

    return trayectoria_muneca

# Ejemplo de uso
hombro_ini = [0, longitud_brazo, 0]
codo_ini = [0, 0, 0]
muneca_ini = [0, -longitud_antebrazo, 0]

grados_flexion_muneca = 100
print(muneca_ini)

trayectoria_muneca = calcular_trayectoria(hombro_ini, codo_ini, muneca_ini, grados_flexion_muneca)

if trayectoria_muneca:
    print("Trayectoria de la muñeca generada con éxito:")
    for punto in trayectoria_muneca:
        print(punto)

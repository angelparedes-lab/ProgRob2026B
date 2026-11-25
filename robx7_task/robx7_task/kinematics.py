"""Cinematica del brazo robx7 (3 articulaciones de giro).

Archivo sin ROS. Lo usa task_manager.py para convertir un punto 3D en angulos.

Geometria tomada de robx7_description/urdf/robx7_arm.xacro
  q1  base_first_joint     giro sobre el eje z, rango de menos pi a pi
  q2  first_second_joint   hombro, rango de menos pi/2 a pi/2
  q3  second_third_joint   codo, rango de menos pi/2 a pi/2

Con q2 = 0 el segundo eslabon apunta hacia arriba. Un q2 positivo lo inclina hacia
adelante. El plano del brazo gira con q1.

Se llama punto de agarre G al punto de la herramienta donde queda el objeto.
Se encuentra a tool_length metros del codo, medidos a lo largo del tercer eslabon
mas la pinza.
"""
import math

# Alturas y longitudes en metros, copiadas del xacro del robot
SHOULDER_HEIGHT = 0.2   # base 0.1 mas primer eslabon 0.1
L2 = 0.4                # longitud del segundo eslabon
DEFAULT_TOOL_LENGTH = 0.46

# Masas y centros de masa tomados de robx7_description/urdf/robx7_arm.xacro y robx7_gripper.xacro
# Cada pareja es masa en kg y distancia en metros desde la articulacion que gira esa parte
GRAVITY = 9.81
SECOND_LINK = (1.0, 0.20)                                  # segundo eslabon, centro a 0.2 del hombro
TOOL_PARTS = ((1.0, 0.15), (1.0, 0.30), (2.0, 0.40))       # tercer eslabon, base de pinza y dos dedos, medidos desde el codo

Q1_LIMITS = (-math.pi, math.pi)
Q2_LIMITS = (-math.pi / 2, math.pi / 2)
Q3_LIMITS = (-math.pi / 2, math.pi / 2)


def forward_kinematics(q1, q2, q3, tool_length=DEFAULT_TOOL_LENGTH):
    """Devuelve el punto (x, y, z) del agarre para unos angulos dados."""
    r = L2 * math.sin(q2) + tool_length * math.sin(q2 + q3)
    z = SHOULDER_HEIGHT + L2 * math.cos(q2) + tool_length * math.cos(q2 + q3)
    return (r * math.cos(q1), r * math.sin(q1), z)


def _in_limits(value, limits, margin=1e-6):
    return limits[0] - margin <= value <= limits[1] + margin


def inverse_kinematics(x, y, z, tool_length=DEFAULT_TOOL_LENGTH):
    """Devuelve (q1, q2, q3) para llevar el punto de agarre a (x, y, z).

    Devuelve None si el punto no es alcanzable o si algun angulo cae fuera de limites.

    Idea en tres pasos
    1. q1 es el angulo del punto visto desde arriba, atan2(y, x).
    2. El problema se vuelve plano con r = distancia horizontal y dz = altura relativa
       al hombro. Es un brazo de dos eslabones, L2 y tool_length.
    3. La ley de cosenos da el codo y la geometria del triangulo da el hombro.
    """
    r = math.hypot(x, y)
    q1 = math.atan2(y, x) if r > 1e-9 else 0.0
    dz = z - SHOULDER_HEIGHT

    d2 = r * r + dz * dz
    cos_q3 = (d2 - L2 * L2 - tool_length * tool_length) / (2.0 * L2 * tool_length)
    if abs(cos_q3) > 1.0:
        return None

    # Dos soluciones de codo. Se devuelve la primera que respeta los limites.
    for sign in (1.0, -1.0):
        q3 = sign * math.acos(cos_q3)
        q2 = math.atan2(r, dz) - math.atan2(tool_length * math.sin(q3),
                                            L2 + tool_length * math.cos(q3))
        if _in_limits(q1, Q1_LIMITS) and _in_limits(q2, Q2_LIMITS) and _in_limits(q3, Q3_LIMITS):
            return (q1, q2, q3)
    return None


def static_joint_torques(q2, q3):
    """Par que la gravedad ejerce sobre hombro y codo en reposo, en N.m.

    Devuelve una pareja con el par del hombro y el del codo, ambos en valor absoluto.
    Si el limite effort del URDF es menor que estos valores, el brazo no puede sostener
    la postura. Se hunde y despues tampoco puede levantarse.

    Para cada articulacion el par es g por la suma de masa por brazo de palanca, y el brazo
    de palanca es la distancia horizontal al eje, o sea la distancia por el seno del angulo.
    Los datos de masas estan arriba y deben actualizarse si se cambian los xacro.
    """
    phi = q2 + q3                                   # inclinacion del tercer eslabon respecto a la vertical
    tool_moment = sum(m * c for m, c in TOOL_PARTS)
    tool_mass = sum(m for m, _ in TOOL_PARTS)
    elbow = GRAVITY * tool_moment * math.sin(phi)
    shoulder = GRAVITY * (SECOND_LINK[0] * SECOND_LINK[1] * math.sin(q2)
                          + tool_mass * L2 * math.sin(q2)
                          + tool_moment * math.sin(phi))
    return abs(shoulder), abs(elbow)

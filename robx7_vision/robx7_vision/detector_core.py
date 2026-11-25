"""Funciones de vision artificial y geometria de la camara.

Archivo sin ROS. Se puede probar con una imagen guardada usando tools/try_core.py
o con las pruebas de test/test_detector_core.py.

LO USA robx7_vision/object_detector.py, que importa este modulo como core.
LO USA robx7_vision/synthetic_data.py y los scripts de la carpeta tools.
"""
from dataclasses import dataclass

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 1. Segmentacion por color en el espacio HSV de OpenCV
#    H va de 0 a 179, S y V van de 0 a 255.
#    El rojo da la vuelta en el circulo de colores, por eso tiene dos rangos.
# ---------------------------------------------------------------------------
COLOR_RANGES = {
    'red': [((0, 150, 40), (8, 255, 255)), ((172, 150, 40), (179, 255, 255))],
    'green': [((40, 150, 40), (80, 255, 255))],
    'blue': [((100, 150, 40), (130, 255, 255))],
}

# Un solo rango que acepta cualquier tono muy saturado. Sirve para proponer candidatos
# sin decidir todavia el color. El color lo decide despues la red neuronal.
ANY_COLOR_RANGES = {'any': [((0, 150, 40), (179, 255, 255))]}


def segment_color(hsv, ranges):
    """Mascara binaria de los pixeles cuyo color cae en alguno de los rangos."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lo), np.array(hi)))
    # La apertura quita ruido pequeno y el cierre rellena huecos
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


# ---------------------------------------------------------------------------
# 2. Contornos, centroide y forma
# ---------------------------------------------------------------------------
@dataclass
class Blob:
    color: str
    cx: float            # centroide [px]
    cy: float
    area: float          # [px^2]
    circularity: float   # 4*pi*A/P^2: circulo ~ 1.0, cuadrado ~ 0.78
    contour: np.ndarray
    shape: str = ''      # lo llena la red neuronal. Vacio significa usar la circularidad


def find_blobs(bgr, min_area=150.0, max_area=3000.0, color_ranges=None):
    """Detecta regiones de color en una imagen BGR."""
    color_ranges = color_ranges or COLOR_RANGES
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blobs = []
    for color, ranges in color_ranges.items():
        mask = segment_color(hsv, ranges)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue
            m = cv2.moments(c)
            if m['m00'] == 0:
                continue
            perimeter = cv2.arcLength(c, True)
            circ = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
            blobs.append(Blob(color, m['m10'] / m['m00'], m['m01'] / m['m00'], area, circ, c))
    return blobs


def classify_shape(circularity, threshold=0.82):
    """Visto desde arriba el cilindro es un circulo y el cubo un cuadrado."""
    return 'cylinder' if circularity >= threshold else 'cube'


# ---------------------------------------------------------------------------
# 3. Geometria. Modelo pinhole y transformacion al marco del robot
# ---------------------------------------------------------------------------
def quat_to_matrix(x, y, z, w):
    """Cuaternion (x, y, z, w) -> matriz de rotacion 3x3."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def intrinsics_from_fov(width, height, hfov):
    """Matriz K de una camara ideal sin distorsion a partir del campo de vision horizontal.

    Es la misma formula con la que Gazebo genera camera_info: fx = (w/2) / tan(hfov/2).
    """
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])


def pixel_to_world(u, v, K, R_wc, t_wc, plane_z):
    """Intersecta el rayo del pixel (u, v) con el plano z igual a plane_z.

    K es la matriz intrinseca de camera_info.
    R_wc y t_wc son la pose del marco optico de la camara expresada en world, que entrega TF2.
    Devuelve el punto 3D en world, o None si el rayo no toca el plano.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])   # direccion del rayo en el marco optico
    d_world = R_wc @ d_cam                                  # la misma direccion en el marco world
    if abs(d_world[2]) < 1e-9:
        return None
    s = (plane_z - t_wc[2]) / d_world[2]                    # t_wc + s*d_world tiene z = plane_z
    if s <= 0:
        return None
    return t_wc + s * d_world


def world_to_pixel(p_world, K, R_wc, t_wc):
    """Proyeccion inversa. Punto 3D en world a pixel (u, v). None si esta detras de la camara."""
    p_cam = R_wc.T @ (np.asarray(p_world, float) - t_wc)
    if p_cam[2] <= 1e-6:
        return None
    return (K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2],
            K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2])

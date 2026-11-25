"""Generacion de datos sinteticos para entrenar y evaluar el clasificador.

Archivo sin ROS. Lo usan tools/make_dataset.py, tools/train_classifier.py y
tools/compare_methods.py, ademas de las pruebas.
USA robx7_vision/detector_core.py para etiquetar recortes reales.

Se dibujan recortes parecidos a los que ve la camara. Cada muestra varia en
  color exacto, tamano, giro, iluminacion, ruido, desenfoque y cara lateral visible.
Esa variacion es lo que permite que la red generalice y no memorice.
"""
import math

import cv2
import numpy as np

from robx7_vision import detector_core as core
from robx7_vision.ai_classifier import CLASSES, crop_contour

TABLE_BGR = (115, 158, 191)
HUE_RANGES = {'red': [(0, 5), (174, 179)], 'green': [(52, 68)], 'blue': [(112, 128)]}


def _object_bgr(color, rng):
    lo, hi = HUE_RANGES[color][rng.integers(len(HUE_RANGES[color]))]
    hsv = np.uint8([[[rng.integers(lo, hi + 1), rng.integers(170, 256), rng.integers(140, 256)]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def render_sample(class_name, rng, gain_range=(0.5, 1.05), noise_max=6.0):
    """Devuelve un recorte BGR uint8 del objeto de la clase pedida."""
    color, shape = class_name.split('_')
    extent = float(rng.uniform(18, 32))            # lado del cubo o diametro del cilindro
    angle = float(rng.uniform(0, 90))
    shift = rng.uniform(-3, 3, 2) if rng.random() < 0.6 else np.zeros(2)   # cara lateral
    margin = 4

    if shape == 'cube':
        half = extent / 2 * (abs(math.cos(math.radians(angle))) + abs(math.sin(math.radians(angle))))
    else:
        half = extent / 2
    side = int(2 * (half + margin + float(np.abs(shift).max()))) + 2
    canvas = np.empty((side, side, 3), np.uint8)
    canvas[:] = [min(255, max(0, c + int(rng.integers(-8, 9)))) for c in TABLE_BGR]

    cx = side / 2 + float(rng.uniform(-1, 1))
    cy = side / 2 + float(rng.uniform(-1, 1))
    top = _object_bgr(color, rng)
    dark = tuple(int(c * 0.65) for c in top)

    def draw(center, bgr):
        if shape == 'cube':
            box = cv2.boxPoints(((center[0], center[1]), (extent, extent), angle))
            cv2.fillPoly(canvas, [np.intp(np.round(box))], bgr)
        else:
            cv2.circle(canvas, (int(round(center[0])), int(round(center[1]))),
                       int(round(extent / 2)), bgr, -1)

    if np.abs(shift).max() > 0:
        draw((cx + shift[0], cy + shift[1]), dark)   # cara lateral, mas oscura y desplazada
    draw((cx, cy), top)

    return degrade(canvas, rng, gain_range, noise_max)


def degrade(img, rng, gain_range=(0.5, 1.05), noise_max=6.0):
    """Aplica iluminacion, desenfoque y ruido a un recorte."""
    out = img.astype(np.float32) * float(rng.uniform(*gain_range))
    if rng.random() < 0.5:
        out = cv2.GaussianBlur(out, (3, 3), 0)
    out += rng.normal(0, float(rng.uniform(0, noise_max)), out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def make_dataset(n_per_class, seed, gain_range=(0.5, 1.05), noise_max=6.0):
    """Devuelve crops (lista de arreglos) y etiquetas enteras."""
    rng = np.random.default_rng(seed)
    crops, labels = [], []
    for idx, name in enumerate(CLASSES):
        for _ in range(n_per_class):
            crops.append(render_sample(name, rng, gain_range, noise_max))
            labels.append(idx)
    return crops, np.array(labels, np.int64)


def augment_template(crop, rng, gain_range=(0.5, 1.05), noise_max=6.0):
    """Variaciones de un recorte real. Giro, escala, iluminacion y ruido."""
    h, w = crop.shape[:2]
    matrix = cv2.getRotationMatrix2D((w / 2, h / 2), float(rng.uniform(0, 360)),
                                     float(rng.uniform(0.9, 1.1)))
    rotated = cv2.warpAffine(crop, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)
    return degrade(rotated, rng, gain_range, noise_max)


def real_templates(frame_bgr):
    """Extrae de un cuadro real un recorte por objeto con su indice de clase.

    Usa el metodo clasico para saber que es cada objeto, y por eso solo sirve con un cuadro
    donde el metodo clasico funciona bien, por ejemplo el guardado con camera_viewer.
    """
    out = []
    for blob in core.find_blobs(frame_bgr):
        name = f'{blob.color}_{core.classify_shape(blob.circularity)}'
        if name in CLASSES:
            out.append((CLASSES.index(name), crop_contour(frame_bgr, blob.contour)))
    return out

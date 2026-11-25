#!/usr/bin/env python3
"""Compara el metodo clasico y la red neuronal sobre los mismos recortes.

Uso
  python3 tools/compare_methods.py --model models/mlp_color_shape.npz

Reporta exactitud, fallas, casos donde cada metodo no responde y tiempo por recorte.
Se evalua en tres condiciones, normal, poca luz y mucho ruido.

LLAMA A robx7_vision/detector_core.py funcion find_blobs y classify_shape
LLAMA A robx7_vision/ai_classifier.py clase CropClassifier
LLAMA A robx7_vision/synthetic_data.py funcion make_dataset
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robx7_vision import ai_classifier as ai  # noqa
from robx7_vision import detector_core as core  # noqa
from robx7_vision import synthetic_data as sd  # noqa

SCENARIOS = {
    'normal': ((0.6, 1.05), 4.0),
    'poca luz': ((0.2, 0.45), 4.0),
    'mucho ruido': ((0.6, 1.05), 25.0),
}


def classic_predict(crop):
    """Metodo clasico. Mascara HSV, contorno y circularidad. Devuelve None si no encuentra nada."""
    padded = cv2.copyMakeBorder(crop, 6, 6, 6, 6, cv2.BORDER_REPLICATE)
    blobs = core.find_blobs(padded, 100.0, 6000.0)
    if not blobs:
        return None
    best = max(blobs, key=lambda b: b.area)
    return best.color, core.classify_shape(best.circularity)


def evaluate(name, predict, crops, labels):
    ok = fail = wrong = 0
    t0 = time.perf_counter()
    for crop, label in zip(crops, labels):
        res = predict(crop)
        if res is None:
            fail += 1
            continue
        if f'{res[0]}_{res[1]}' == ai.CLASSES[label]:
            ok += 1
        else:
            wrong += 1
    ms = (time.perf_counter() - t0) * 1000.0 / len(labels)
    n = len(labels)
    print(f'  {name:8s} exactitud {ok / n:6.1%}  sin respuesta {fail / n:6.1%}  '
          f'equivocados {wrong / n:6.1%}  tiempo {ms:6.3f} ms por recorte')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='models/mlp_color_shape.npz')
    parser.add_argument('--n', type=int, default=200, help='muestras por clase')
    args = parser.parse_args()

    clf = ai.CropClassifier.load(args.model)

    def ai_predict(crop):
        color, shape, _ = clf.predict(crop)
        return color, shape

    for title, (gain, noise) in SCENARIOS.items():
        crops, labels = sd.make_dataset(args.n, 777, gain, noise)   # semilla distinta a la del entrenamiento
        print(f'Condicion {title}, {len(labels)} recortes')
        evaluate('clasico', classic_predict, crops, labels)
        evaluate('red', ai_predict, crops, labels)


if __name__ == '__main__':
    main()

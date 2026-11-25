#!/usr/bin/env python3
"""Genera un conjunto de datos de recortes y lo guarda en un archivo npz.

Uso
  python3 tools/make_dataset.py --n 300 --out /tmp/dataset.npz
  python3 tools/make_dataset.py --n 300 --frame /tmp/frame.png --out /tmp/dataset.npz

Con la opcion frame, parte de los recortes se obtienen del cuadro real de la camara
guardado en la sesion anterior y se multiplican con variaciones.

LLAMA A robx7_vision/synthetic_data.py funcion make_dataset y augment_template
LLAMA A robx7_vision/detector_core.py funcion find_blobs
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robx7_vision import ai_classifier as ai  # noqa
from robx7_vision import synthetic_data as sd  # noqa


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=int, default=300, help='muestras por clase')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--gain-min', type=float, default=0.25)
    parser.add_argument('--gain-max', type=float, default=1.05)
    parser.add_argument('--noise', type=float, default=6.0)
    parser.add_argument('--frame', default='', help='cuadro real opcional')
    parser.add_argument('--out', default='/tmp/dataset.npz')
    args = parser.parse_args()

    gain = (args.gain_min, args.gain_max)
    crops, labels = sd.make_dataset(args.n, args.seed, gain, args.noise)
    if args.frame:
        frame = cv2.imread(args.frame)
        if frame is None:
            sys.exit('No se pudo leer ' + args.frame)
        rng = np.random.default_rng(args.seed + 100)
        for idx, tmpl in sd.real_templates(frame):
            for _ in range(args.n):
                crops.append(sd.augment_template(tmpl, rng, gain, args.noise))
                labels = np.append(labels, idx)
        print('Se agregaron recortes derivados del cuadro real')

    feats = np.stack([ai.extract_features(c) for c in crops])
    np.savez(args.out, x=feats, y=labels, classes=np.array(ai.CLASSES))
    print(f'Guardado {args.out} con {len(labels)} muestras y {feats.shape[1]} entradas')


if __name__ == '__main__':
    main()

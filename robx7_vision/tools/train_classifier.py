#!/usr/bin/env python3
"""Entrena el clasificador de color y forma y guarda el modelo.

Uso
  python3 tools/train_classifier.py
  python3 tools/train_classifier.py --frame /tmp/frame.png --out models/mlp_color_shape.npz

El resultado se instala con el paquete y lo carga object_detector.py con model_path.

LLAMA A robx7_vision/synthetic_data.py
LLAMA A robx7_vision/ai_classifier.py clase MLP
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robx7_vision import ai_classifier as ai  # noqa
from robx7_vision import synthetic_data as sd  # noqa


def build(n, seed, gain, noise, frame):
    crops, labels = sd.make_dataset(n, seed, gain, noise)
    if frame:
        img = cv2.imread(frame)
        if img is None:
            sys.exit('No se pudo leer ' + frame)
        rng = np.random.default_rng(seed + 100)
        for idx, tmpl in sd.real_templates(img):
            for _ in range(n):
                crops.append(sd.augment_template(tmpl, rng, gain, noise))
                labels = np.append(labels, idx)
    return np.stack([ai.extract_features(c) for c in crops]), labels


def accuracy(net, x, y):
    return float((net.predict_proba(x).argmax(axis=1) == y).mean())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-train', type=int, default=600)
    parser.add_argument('--n-test', type=int, default=150)
    parser.add_argument('--epochs', type=int, default=40)
    parser.add_argument('--hidden', type=int, default=32)
    parser.add_argument('--frame', default='')
    parser.add_argument('--out', default='models/mlp_color_shape.npz')
    args = parser.parse_args()

    gain = (0.25, 1.05)
    x_tr, y_tr = build(args.n_train, 1, gain, 6.0, args.frame)
    x_te, y_te = build(args.n_test, 2, gain, 6.0, args.frame)    # semilla distinta, nunca vistos
    print(f'Entrenamiento {len(y_tr)} muestras, prueba {len(y_te)} muestras')

    net = ai.MLP(x_tr.shape[1], args.hidden, len(ai.CLASSES))
    t0 = time.time()

    def report(epoch, loss):
        if epoch == 1 or epoch % 10 == 0:
            print(f'  epoca {epoch:3d}  perdida {loss:.4f}  exactitud prueba {accuracy(net, x_te, y_te):.3f}')

    net.fit(x_tr, y_tr, epochs=args.epochs, on_epoch=report)
    print(f'Tiempo de entrenamiento {time.time() - t0:.1f} s')
    print(f'Exactitud entrenamiento {accuracy(net, x_tr, y_tr):.3f}  prueba {accuracy(net, x_te, y_te):.3f}')

    pred = net.predict_proba(x_te).argmax(axis=1)
    print('Matriz de confusion, filas reales y columnas predichas')
    print('            ' + ' '.join(f'{c[:6]:>6s}' for c in ai.CLASSES))
    for i, name in enumerate(ai.CLASSES):
        row = [int(((y_te == i) & (pred == j)).sum()) for j in range(len(ai.CLASSES))]
        print(f'{name:12s}' + ' '.join(f'{v:6d}' for v in row))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    net.save(args.out)
    print('Modelo guardado en', args.out)


if __name__ == '__main__':
    main()

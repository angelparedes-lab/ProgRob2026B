"""Pruebas del clasificador con red neuronal. No necesitan ROS ni Gazebo."""
import os

import numpy as np

from robx7_vision import ai_classifier as ai
from robx7_vision import synthetic_data as sd


def test_features_shape_and_range():
    crop = sd.render_sample('red_cube', np.random.default_rng(0))
    feats = ai.extract_features(crop)
    assert feats.shape == (ai.IMG_SIZE * ai.IMG_SIZE * 3,)
    assert 0.0 <= feats.min() and feats.max() <= 1.0


def test_small_network_learns(tmp_path):
    crops, y = sd.make_dataset(100, 5, (0.6, 1.05), 3.0)
    x = np.stack([ai.extract_features(c) for c in crops])
    net = ai.MLP(x.shape[1], 32, len(ai.CLASSES))
    net.fit(x, y, epochs=60)
    acc = float((net.predict_proba(x).argmax(axis=1) == y).mean())
    # el azar acierta una de seis veces, 0.17, asi que 0.7 demuestra que aprendio
    assert acc > 0.7
    # guardar y cargar no debe cambiar las predicciones
    path = os.path.join(str(tmp_path), 'net.npz')
    net.save(path)
    again = ai.MLP.load(path)
    assert np.allclose(net.predict_proba(x[:10]), again.predict_proba(x[:10]), atol=1e-6)


def test_shipped_model_accuracy():
    model_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'mlp_color_shape.npz')
    if not os.path.exists(model_path):
        return
    clf = ai.CropClassifier.load(model_path)
    crops, y = sd.make_dataset(30, 4242, (0.6, 1.05), 4.0)
    hits = 0
    for crop, label in zip(crops, y):
        color, shape, _ = clf.predict(crop)
        hits += int(f'{color}_{shape}' == ai.CLASSES[label])
    assert hits / len(y) > 0.85

"""Pruebas de la evaluacion de salud. No necesitan ROS."""
import importlib.util
import os

spec = importlib.util.spec_from_file_location(
    'salud_ros', os.path.join(os.path.dirname(__file__), '..', 'tools', 'salud_ros.py'))
salud = importlib.util.module_from_spec(spec)
spec.loader.exec_module(salud)


def healthy():
    return {'clock_publishers': 1, 'clock_msgs': 2000, 'clock_backward_jumps': 0,
            'clock_rate_wall': 330.0, 'rtf': 0.95, 'image_publishers': 1, 'image_msgs': 60,
            'image_rate_wall': 10.0, 'objects_publishers': 1, 'objects_msgs': 30,
            'objects_rate_wall': 5.0, 'duplicated_nodes': [], 'n_nodes': 15}


def levels(stats):
    return [lvl for lvl, _ in salud.evaluate(stats)]


def test_healthy_system_has_no_failures():
    assert set(levels(healthy())) == {'OK'}


def test_two_clocks_are_detected():
    s = healthy()
    s['clock_publishers'] = 2
    s['clock_backward_jumps'] = 5
    texts = ' '.join(t for _, t in salud.evaluate(s))
    assert 'FALLO' in levels(s) and 'retrocedio' in texts and 'otro equipo' in texts


def test_slow_simulation_and_silent_detector():
    s = healthy()
    s['rtf'] = 0.2
    s['objects_msgs'] = 0
    lv = levels(s)
    assert lv.count('FALLO') == 2


def test_duplicated_nodes():
    s = healthy()
    s['duplicated_nodes'] = ['/object_detector']
    assert 'FALLO' in levels(s)

"""Pruebas de la cinematica. No necesitan ROS ni Gazebo."""
import math

from robx7_task import kinematics as k


def test_forward_home():
    x, y, z = k.forward_kinematics(0.0, 0.0, math.pi / 2, 0.46)
    assert abs(x - 0.46) < 1e-9 and abs(y) < 1e-9 and abs(z - 0.6) < 1e-9


def test_ik_roundtrip_on_scene_points():
    for x, y, z in [(0.66, 0.08, 0.045), (0.74, -0.07, 0.045), (0.64, -0.14, 0.145),
                    (0.5, 0.5, 0.13), (0.0, 0.7, 0.13)]:
        q = k.inverse_kinematics(x, y, z)
        assert q is not None, (x, y, z)
        fx, fy, fz = k.forward_kinematics(*q)
        assert abs(fx - x) < 1e-6 and abs(fy - y) < 1e-6 and abs(fz - z) < 1e-6


def test_ik_unreachable_points():
    assert k.inverse_kinematics(1.5, 0.0, 0.05) is None     # demasiado lejos
    assert k.inverse_kinematics(0.05, 0.0, 0.05) is None    # demasiado cerca de la base


def test_ik_respects_limits():
    q = k.inverse_kinematics(0.7, 0.0, 0.045)
    assert q is not None
    assert k.Q2_LIMITS[0] <= q[1] <= k.Q2_LIMITS[1]
    assert k.Q3_LIMITS[0] <= q[2] <= k.Q3_LIMITS[1]


def test_static_torque_values():
    # En HOME solo carga el tercer eslabon con la pinza, unos 12 N.m
    shoulder, elbow = k.static_joint_torques(0.0, math.pi / 2)
    assert abs(shoulder - 12.26) < 0.1 and abs(elbow - 12.26) < 0.1
    # Brazo horizontal es el peor caso, cerca de 30 N.m
    worst, _ = k.static_joint_torques(math.pi / 2, 0.0)
    assert abs(worst - 29.9) < 0.2


def test_task_poses_need_more_than_the_old_effort_limit():
    # Explica la caida del brazo. Con el limite original de 20 N.m estas posturas no se sostienen
    q = k.inverse_kinematics(0.66, 0.08, 0.045)
    shoulder, _ = k.static_joint_torques(q[1], q[2])
    assert 20.0 < shoulder < 100.0

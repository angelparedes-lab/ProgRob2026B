#!/usr/bin/env python3
"""Mide durante unos segundos la salud de la cadena Gazebo y ROS 2.

Uso
  python3 tools/salud_ros.py
  python3 tools/salud_ros.py 10

Con el sistema lanzado revisa cuantos relojes hay, a que velocidad corre la simulacion,
si llegan imagenes y detecciones, y si hay nodos duplicados en la red.
La funcion evaluate no usa ROS y se prueba aparte en test/test_salud.py.
"""
import sys
import time
from collections import Counter


def evaluate(stats):
    """Convierte las mediciones en una lista de parejas nivel y texto.

    El nivel es OK, AVISO o FALLO.
    """
    out = []
    n_clk = stats['clock_publishers']
    if n_clk == 0:
        out.append(('FALLO', 'Nadie publica /clock. Gazebo no esta corriendo o el puente de reloj fallo'))
    elif n_clk > 1:
        out.append(('FALLO', f'Hay {n_clk} publicadores de /clock. Quedaron procesos de Gazebo de otra '
                             f'ejecucion o hay otro equipo en tu red con el mismo ROS_DOMAIN_ID'))
    else:
        out.append(('OK', 'Un solo publicador de /clock'))

    if stats['clock_backward_jumps'] > 0:
        out.append(('FALLO', f'El reloj de simulacion retrocedio {stats["clock_backward_jumps"]} veces. '
                             f'Hay dos relojes mezclados, el sistema se comporta de forma erratica'))

    rtf = stats['rtf']
    if stats['clock_msgs'] == 0:
        out.append(('FALLO', 'No llego ningun mensaje de /clock en el tiempo de medicion'))
    elif rtf < 0.3:
        out.append(('FALLO', f'La simulacion corre al {rtf:.0%} del tiempo real. El equipo esta saturado'))
    elif rtf < 0.8:
        out.append(('AVISO', f'La simulacion corre al {rtf:.0%} del tiempo real. Va justa de recursos'))
    else:
        out.append(('OK', f'La simulacion corre al {rtf:.0%} del tiempo real'))

    if stats['clock_rate_wall'] > 600:
        out.append(('AVISO', f'/clock llega a {stats["clock_rate_wall"]:.0f} mensajes por segundo. '
                             f'Es mucho trafico. Un paso de fisica mayor lo reduce'))

    if stats['image_publishers'] == 0:
        out.append(('FALLO', 'Nadie publica /camera/image_raw. Revisa el puente de la camara'))
    elif stats['image_msgs'] == 0:
        out.append(('FALLO', 'No llegaron imagenes en el tiempo de medicion'))
    else:
        out.append(('OK', f'Llegan imagenes a {stats["image_rate_wall"]:.1f} por segundo reales'))

    n_det = stats['objects_publishers']
    if n_det == 0:
        out.append(('FALLO', 'Nadie publica /vision/objects. El nodo object_detector no corre'))
    elif n_det > 1:
        out.append(('FALLO', f'Hay {n_det} publicadores de /vision/objects. Otro proceso o equipo comparte el dominio'))
    elif stats['objects_msgs'] == 0:
        out.append(('FALLO', 'object_detector existe pero no publico en el tiempo de medicion. '
                             'Revisa camera_info, TF y el aviso de su terminal'))
    else:
        out.append(('OK', f'object_detector publica {stats["objects_rate_wall"]:.1f} mensajes por segundo reales'))

    if stats['duplicated_nodes']:
        out.append(('FALLO', 'Nodos duplicados en la red ' + ', '.join(stats['duplicated_nodes']) +
                    '. Hay dos lanzamientos activos'))
    return out


def measure(seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from robx7_interfaces.msg import DetectedObjectArray
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    class Probe(Node):
        def __init__(self):
            super().__init__('salud_ros')
            self.clock_msgs = 0
            self.first_sim = None
            self.last_sim = None
            self.backward = 0
            self.images = 0
            self.objects = 0
            self.create_subscription(Clock, '/clock', self.on_clock, 100)
            self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
            self.create_subscription(DetectedObjectArray, '/vision/objects', self.on_objects, 10)

        def on_clock(self, msg):
            sim = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.first_sim is None:
                self.first_sim = sim
            if self.last_sim is not None and sim < self.last_sim - 1e-6:
                self.backward += 1
            self.last_sim = sim
            self.clock_msgs += 1

        def on_image(self, msg):
            self.images += 1

        def on_objects(self, msg):
            self.objects += 1

    rclpy.init()
    node = Probe()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.monotonic() - t0

    names = [f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces() if n != 'salud_ros']
    duplicated = sorted(name for name, count in Counter(names).items() if count > 1)
    sim_elapsed = (node.last_sim - node.first_sim) if node.first_sim is not None else 0.0
    stats = {
        'clock_publishers': node.count_publishers('/clock'),
        'clock_msgs': node.clock_msgs,
        'clock_backward_jumps': node.backward,
        'clock_rate_wall': node.clock_msgs / wall,
        'rtf': sim_elapsed / wall,
        'image_publishers': node.count_publishers('/camera/image_raw'),
        'image_msgs': node.images,
        'image_rate_wall': node.images / wall,
        'objects_publishers': node.count_publishers('/vision/objects'),
        'objects_msgs': node.objects,
        'objects_rate_wall': node.objects / wall,
        'duplicated_nodes': duplicated,
        'n_nodes': len(names),
    }
    node.destroy_node()
    rclpy.shutdown()
    return stats, wall


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    print(f'Midiendo durante {seconds:.0f} s ...')
    stats, wall = measure(seconds)
    print(f'Nodos visibles {stats["n_nodes"]}, mensajes de reloj {stats["clock_msgs"]} en {wall:.1f} s')
    for level, text in evaluate(stats):
        print(f'  [{level:5s}] {text}')


if __name__ == '__main__':
    main()

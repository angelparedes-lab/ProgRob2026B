"""Nodo gestor de tarea. Maquina de estados finitos de recoger y clasificar objetos.

RELACIONES DE ESTE ARCHIVO
  LO LANZA     robx7_bringup/launch/robx7_cell.launch.xml
  LEE PARAMS   robx7_task/config/task.yaml
  USA          robx7_task/robx7_task/fsm.py  clase StateMachine
  USA          robx7_task/robx7_task/kinematics.py  funcion inverse_kinematics
  ESCUCHA      /vision/objects publicado por robx7_vision/robx7_vision/object_detector.py
  ESCUCHA      /joint_states publicado por joint_state_broadcaster
  PUBLICA      /brazo_position_controller/commands que define robx7_description/config/robx7_gazebo_config.yaml
  PUBLICA      /gripper_position_controller/commands definido en el mismo archivo

Estados del proyecto final
  HOME, DETECTAR, APROXIMAR, TOMAR, TRANSPORTAR, SOLTAR, VERIFICAR, RECUPERACION, FIN
"""
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robx7_interfaces.msg import DetectedObjectArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

# LLAMADA A ARCHIVO robx7_task/robx7_task/fsm.py
from robx7_task.fsm import StateMachine
# LLAMADA A ARCHIVO robx7_task/robx7_task/kinematics.py
from robx7_task.kinematics import inverse_kinematics, static_joint_torques

# Nombres de articulaciones, deben coincidir con robx7_description/config/robx7_gazebo_config.yaml
ARM_JOINTS = ['base_first_joint', 'first_second_joint', 'second_third_joint']
FINGER_JOINT = 'gripper_base_finger_left'


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Parametros. Sus valores llegan desde robx7_task/config/task.yaml
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('max_recoveries', 10)
        self.declare_parameter('home', [0.0, 0.0, 1.5])
        self.declare_parameter('effort_limit', 100.0)
        self.declare_parameter('check_torque', True)
        self.declare_parameter('tool_length', 0.46)
        self.declare_parameter('grasp_height', 0.045)
        self.declare_parameter('approach_height', 0.12)
        self.declare_parameter('carry_height', 0.16)
        self.declare_parameter('drop_height', 0.13)
        self.declare_parameter('grip_open', 0.0)
        self.declare_parameter('grip_close', 0.06)
        self.declare_parameter('check_grasp', True)
        self.declare_parameter('grip_empty_threshold', 0.065)
        self.declare_parameter('move_time', 2.0)
        self.declare_parameter('settle_wait', 1.0)
        self.declare_parameter('detect_timeout', 10.0)
        for name, default in (('red', [0.5, 0.5]), ('green', [0.5, -0.5]), ('blue', [0.0, 0.7]),
                              ('cube', [0.5, 0.5]), ('cylinder', [0.5, -0.5])):
            self.declare_parameter(f'dest_{name}', default)

        gp = self.get_parameter
        self.auto_start = gp('auto_start').value
        self.max_retries = gp('max_retries').value
        self.max_recoveries = gp('max_recoveries').value
        self.home = list(gp('home').value)
        self.effort_limit = gp('effort_limit').value
        self.check_torque = gp('check_torque').value
        self.tool = gp('tool_length').value
        self.h_grasp = gp('grasp_height').value
        self.h_approach = gp('approach_height').value
        self.h_carry = gp('carry_height').value
        self.h_drop = gp('drop_height').value
        self.grip_open = gp('grip_open').value
        self.grip_close = gp('grip_close').value
        self.check_grasp = gp('check_grasp').value
        self.grip_empty = gp('grip_empty_threshold').value
        self.move_time = gp('move_time').value
        self.settle_wait = gp('settle_wait').value
        self.detect_timeout = gp('detect_timeout').value
        self.destinations = {n: list(gp(f'dest_{n}').value)
                             for n in ('red', 'green', 'blue', 'cube', 'cylinder')}

        # Datos que llegan de otros nodos
        self.latest = None          # ultimo mensaje de objetos
        self.objects_seq = 0        # contador de mensajes de objetos recibidos
        self.last_objects_wall = None   # instante en tiempo real del ultimo mensaje
        self.joint_pos = {}         # posicion actual por nombre de articulacion

        # Estado de la corrida
        self.start_event = threading.Event()
        self.running = False
        self.reset_run()

        # Salidas hacia los controladores. Ver robx7_description/config/robx7_gazebo_config.yaml
        self.pub_arm = self.create_publisher(Float64MultiArray,
                                             '/brazo_position_controller/commands', 10)
        self.pub_grip = self.create_publisher(Float64MultiArray,
                                              '/gripper_position_controller/commands', 10)
        # Salidas informativas para RViz y para el reporte
        self.pub_state = self.create_publisher(String, '/task/state', 10)
        self.pub_report = self.create_publisher(String, '/task/report', 10)
        self.pub_marker = self.create_publisher(Marker, '/task/state_marker', 10)

        # Entradas. Los callbacks corren en el hilo del executor
        self.create_subscription(DetectedObjectArray, '/vision/objects', self.on_objects, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_service(Trigger, '/task/start', self.on_start)
        self.create_timer(1.0, self.publish_status)

        # La maquina de estados. Cada estado es un metodo de esta clase
        self.fsm = StateMachine('HOME', 'FIN', on_transition=self.on_transition)
        self.fsm.add_state('HOME', self.state_home)
        self.fsm.add_state('DETECTAR', self.state_detect)
        self.fsm.add_state('APROXIMAR', self.state_approach)
        self.fsm.add_state('TOMAR', self.state_grasp)
        self.fsm.add_state('TRANSPORTAR', self.state_transport)
        self.fsm.add_state('SOLTAR', self.state_release)
        self.fsm.add_state('VERIFICAR', self.state_verify)
        self.fsm.add_state('RECUPERACION', self.state_recovery)
        self.fsm.add_state('FIN', self.state_end)

        self.get_logger().info('Gestor de tarea listo. Inicia con ros2 service call /task/start std_srvs/srv/Trigger')

    # ------------------------------------------------------------------ utilidades
    def reset_run(self):
        self.target = None          # objeto que se esta manipulando
        self.plan = None            # angulos precalculados para el objeto
        self.reason = ''            # causa de la ultima falla
        self.retries = {}           # intentos fallidos por id
        self.blacklist = set()      # ids descartados
        self.done = []              # ids colocados con exito
        self.failures = []          # lista de id y causa
        self.recoveries = 0
        self.t_run = self.now()
        self.t_object = self.t_run
        self.object_times = []

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds):
        """Espera segun el reloj de simulacion, con un tope en tiempo real por seguridad."""
        t0 = self.now()
        wall0 = time.monotonic()
        while rclpy.ok() and (self.now() - t0) < seconds:
            if time.monotonic() - wall0 > seconds * 20.0 + 5.0:
                break
            time.sleep(0.01)

    def on_objects(self, msg):
        self.latest = msg
        self.objects_seq += 1
        self.last_objects_wall = time.monotonic()

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    def on_start(self, request, response):
        if self.running:
            response.success = False
            response.message = 'La tarea ya esta en ejecucion'
        else:
            self.start_event.set()
            response.success = True
            response.message = 'Tarea iniciada'
        return response

    def on_transition(self, previous, new):
        self.get_logger().info(f'Transicion {previous} hacia {new}')
        self.publish_status()

    def publish_status(self):
        state = self.fsm.current if self.running else 'ESPERANDO INICIO'
        obj = self.target['id'] if self.target else 'ninguno'
        text = f'FSM {state}\nobjeto {obj}'
        self.pub_state.publish(String(data=state))
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'task', 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.3, 0.0, 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.2, 1.0
        m.text = text
        self.pub_marker.publish(m)

    def fail(self, reason):
        """Registra la causa de una falla y manda la maquina a RECUPERACION."""
        self.reason = reason
        self.get_logger().warn(f'Falla. {reason}')
        return 'RECUPERACION'

    # ------------------------------------------------------------------ movimiento
    def arm_now(self):
        if not all(j in self.joint_pos for j in ARM_JOINTS):
            return None
        return [self.joint_pos[j] for j in ARM_JOINTS]

    def move_arm(self, q_target, duration):
        """Mueve el brazo en espacio articular con una curva suave de cinco grados."""
        q0 = self.arm_now()
        if q0 is None:
            self.reason = 'sin joint_states, los controladores no estan activos'
            return False
        t0 = self.now()
        while rclpy.ok():
            s = min(1.0, (self.now() - t0) / duration)
            k = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = [a + (b - a) * k for a, b in zip(q0, q_target)]
            self.pub_arm.publish(Float64MultiArray(data=[float(v) for v in q]))
            if s >= 1.0:
                break
            time.sleep(0.02)
        return self.wait_arm(q_target)

    def wait_arm(self, q_target, tol=0.03, timeout=3.0):
        t0 = self.now()
        while rclpy.ok() and (self.now() - t0) < timeout:
            q = self.arm_now()
            if q is not None and max(abs(a - b) for a, b in zip(q, q_target)) < tol:
                return True
            time.sleep(0.02)
        self.reason = 'el brazo no alcanzo la posicion pedida'
        return False

    def set_gripper(self, opening, duration=0.8):
        """Cierra o abre la pinza. El valor es el desplazamiento de cada dedo en metros."""
        q0 = self.joint_pos.get(FINGER_JOINT, 0.0)
        t0 = self.now()
        while rclpy.ok():
            s = min(1.0, (self.now() - t0) / duration)
            q = q0 + (opening - q0) * s
            self.pub_grip.publish(Float64MultiArray(data=[float(q), float(q)]))
            if s >= 1.0:
                break
            time.sleep(0.02)

    def wait_fresh_objects(self):
        """Espera un mensaje de objetos nuevo.

        Se cuenta por numero de mensaje y el limite se mide en tiempo real. Asi no depende
        del reloj de simulacion, que puede detenerse o saltar si Gazebo va lento o si hay
        otro reloj publicandose en la red.
        """
        seq0 = self.objects_seq
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < self.detect_timeout:
            if self.objects_seq > seq0:
                return self.latest
            time.sleep(0.05)
        return None

    def perception_reason(self):
        """Arma una explicacion de por que no llegaron detecciones, con datos del grafo ROS."""
        parts = [f'la percepcion no entrego datos en {self.detect_timeout:.0f} s']
        n_det = self.count_publishers('/vision/objects')
        n_clk = self.count_publishers('/clock')
        if n_det == 0:
            parts.append('nadie publica /vision/objects, el nodo object_detector no esta corriendo')
        elif n_det > 1:
            parts.append(f'hay {n_det} publicadores de /vision/objects, otro proceso o equipo comparte el dominio ROS')
        if n_clk > 1:
            parts.append(f'hay {n_clk} publicadores de /clock, quedaron procesos de Gazebo de otra ejecucion '
                         f'o hay otro equipo en la red')
        if self.last_objects_wall is None:
            parts.append('nunca llego un mensaje de objetos')
        else:
            parts.append(f'el ultimo mensaje llego hace {time.monotonic() - self.last_objects_wall:.0f} s')
        parts.append('ejecuta robx7_vision/tools/salud_ros.py para ver la causa')
        return '. '.join(parts)

    def check_environment(self):
        """Avisa al iniciar si se detecta algo que suele causar fallas de percepcion."""
        n_clk = self.count_publishers('/clock')
        if n_clk > 1:
            self.get_logger().warn(
                f'Hay {n_clk} publicadores de /clock. Probablemente quedaron procesos de Gazebo '
                f'de una ejecucion anterior. Cierra todo y ejecuta robx7_vision/tools/limpiar_procesos.sh')

    # ------------------------------------------------------------------ estados
    def state_home(self):
        """HOME. Postura segura que deja libre la vista de la camara."""
        self.set_gripper(self.grip_open)
        if not self.move_arm(self.home, self.move_time):
            return self.fail(self.reason)
        self.sleep(self.settle_wait)
        return 'DETECTAR'

    def state_detect(self):
        """DETECTAR. Pide las detecciones y elige el objeto mas cercano a la base."""
        msg = self.wait_fresh_objects()
        if msg is None:
            return self.fail(self.perception_reason())

        candidates = [o for o in msg.objects if o.id not in self.blacklist]
        candidates.sort(key=lambda o: math.hypot(o.pose.position.x, o.pose.position.y))
        for obj in candidates:
            plan = self.make_plan(obj)
            if plan is None:
                self.get_logger().warn(f'{obj.id} fuera de alcance o sin destino, se descarta')
                self.blacklist.add(obj.id)
                self.failures.append((obj.id, 'fuera de alcance o sin destino'))
                continue
            if self.target is None or self.target['id'] != obj.id:
                self.t_object = self.now()
            self.target = {'id': obj.id, 'label': obj.label,
                           'x': obj.pose.position.x, 'y': obj.pose.position.y}
            self.plan = plan
            self.get_logger().info(
                f'Objeto elegido {obj.id} clase {obj.label} en x {self.target["x"]:.3f} y {self.target["y"]:.3f}')
            return 'APROXIMAR'
        return 'FIN'

    def make_plan(self, obj):
        """Calcula con cinematica inversa todos los angulos que hacen falta. None si algo no llega."""
        dest = self.destinations.get(obj.label)
        if dest is None:
            return None
        x, y = obj.pose.position.x, obj.pose.position.y
        poses = {
            'approach': (x, y, self.h_approach),
            'grasp': (x, y, self.h_grasp),
            'carry': (x, y, self.h_carry),
            'drop': (dest[0], dest[1], self.h_drop),
        }
        plan = {}
        for name, (px, py, pz) in poses.items():
            # LLAMADA A ARCHIVO robx7_task/robx7_task/kinematics.py
            q = inverse_kinematics(px, py, pz, self.tool)
            if q is None:
                return None
            if self.check_torque and not self.holdable(obj.id, name, q):
                return None
            plan[name] = list(q)
        return plan

    def holdable(self, obj_id, pose_name, q):
        """Comprueba que el brazo pueda sostener la postura contra la gravedad."""
        shoulder, elbow = static_joint_torques(q[1], q[2])
        worst = max(shoulder, elbow)
        if worst > self.effort_limit:
            self.get_logger().warn(
                f'La postura {pose_name} de {obj_id} exige {worst:.1f} N.m y el limite es '
                f'{self.effort_limit:.1f} N.m. El brazo no la sostendria. '
                f'Sube arm_effort en robx7_description/urdf/common_properties.xacro',
                throttle_duration_sec=10.0)
            return False
        return True

    def state_approach(self):
        """APROXIMAR. Se coloca sobre el objeto con la pinza abierta."""
        self.set_gripper(self.grip_open)
        if not self.move_arm(self.plan['approach'], self.move_time):
            return self.fail(self.reason)
        return 'TOMAR'

    def state_grasp(self):
        """TOMAR. Desciende, cierra la pinza, levanta y confirma el agarre."""
        if not self.move_arm(self.plan['grasp'], self.move_time * 0.6):
            return self.fail(self.reason)
        self.set_gripper(self.grip_close, 1.0)
        self.sleep(0.4)
        if not self.move_arm(self.plan['carry'], self.move_time * 0.6):
            return self.fail(self.reason)
        if self.check_grasp:
            opening = self.joint_pos.get(FINGER_JOINT, 0.0)
            if opening >= self.grip_empty:
                return self.fail('agarre fallido, la pinza cerro sin objeto')
        return 'TRANSPORTAR'

    def state_transport(self):
        """TRANSPORTAR. Lleva el objeto sobre el destino de su clase."""
        if not self.move_arm(self.plan['drop'], self.move_time * 1.5):
            return self.fail(self.reason)
        return 'SOLTAR'

    def state_release(self):
        """SOLTAR. Abre la pinza y deja caer el objeto."""
        self.set_gripper(self.grip_open)
        self.sleep(0.6)
        return 'VERIFICAR'

    def state_verify(self):
        """VERIFICAR. Regresa a HOME, mira de nuevo la mesa y comprueba el resultado."""
        if not self.move_arm(self.home, self.move_time):
            return self.fail(self.reason)
        self.sleep(self.settle_wait)
        msg = self.wait_fresh_objects()
        if msg is None:
            return self.fail(self.perception_reason())
        ids = [o.id for o in msg.objects]
        if self.target['id'] in ids:
            return self.fail('el objeto sigue en la zona de entrada')
        self.done.append(self.target['id'])
        self.object_times.append(self.now() - self.t_object)
        self.retries.pop(self.target['id'], None)
        self.get_logger().info(f'Objeto {self.target["id"]} colocado correctamente')
        remaining = [i for i in ids if i not in self.blacklist]
        return 'DETECTAR' if remaining else 'FIN'

    def state_recovery(self):
        """RECUPERACION. Reintenta de forma acotada, descarta el objeto o termina."""
        self.recoveries += 1
        self.set_gripper(self.grip_open)
        if self.target is not None:
            oid = self.target['id']
            self.failures.append((oid, self.reason))
            self.retries[oid] = self.retries.get(oid, 0) + 1
            if self.retries[oid] > self.max_retries:
                self.blacklist.add(oid)
                self.get_logger().warn(f'{oid} descartado tras {self.max_retries} reintentos')
        if self.recoveries > self.max_recoveries:
            self.get_logger().error('Demasiadas recuperaciones, se termina la tarea')
            return 'FIN'
        return 'HOME'

    def state_end(self):
        """FIN. Deja el brazo en HOME y publica las metricas de la corrida."""
        self.set_gripper(self.grip_open)
        self.move_arm(self.home, self.move_time)
        total = self.now() - self.t_run
        avg = sum(self.object_times) / len(self.object_times) if self.object_times else 0.0
        report = (f'REPORTE colocados {len(self.done)} {self.done} | '
                  f'descartados {sorted(self.blacklist)} | recuperaciones {self.recoveries} | '
                  f'tiempo total {total:.1f} s | tiempo medio por objeto {avg:.1f} s')
        self.get_logger().info(report)
        self.pub_report.publish(String(data=report))
        return 'FIN'

    # ------------------------------------------------------------------ bucle principal
    def run_forever(self):
        """Espera la orden de inicio y ejecuta la maquina. Corre en el hilo principal."""
        if self.auto_start:
            self.start_event.set()
        while rclpy.ok():
            if not self.start_event.wait(timeout=0.5):
                continue
            self.start_event.clear()
            self.reset_run()
            self.check_environment()
            self.running = True
            self.publish_status()
            self.fsm.run(should_continue=rclpy.ok)
            self.running = False
            self.publish_status()
            self.get_logger().info('Tarea terminada. Llama de nuevo a /task/start para repetir')


def main():
    rclpy.init()
    node = TaskManager()
    # El executor atiende suscripciones y servicios en un hilo aparte.
    # El hilo principal queda libre para ejecutar la maquina de estados de forma lineal.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

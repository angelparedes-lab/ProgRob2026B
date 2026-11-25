"""Nodo de percepcion.

Flujo cada vez que llega una imagen
  imagen, blobs de color, clase, pixel, punto 3D en world con camera_info y TF2, salidas.

RELACIONES DE ESTE ARCHIVO
  LO LANZA     robx7_bringup/launch/robx7_cell.launch.xml
  USA          robx7_vision/robx7_vision/detector_core.py  metodo clasico y geometria
  USA          robx7_vision/robx7_vision/ai_classifier.py  metodo con red neuronal
  LEE          robx7_vision/config/ground_truth.yaml  posiciones reales para medir el error
  LEE          robx7_vision/models/mlp_color_shape.npz  pesos de la red entrenada
  ESCUCHA      /camera/image_raw y /camera/camera_info que salen del puente de Gazebo
  ESCUCHA      TF entre world y camera_optical_link que publica robot_state_publisher
  PUBLICA      /vision/objects con el tipo definido en robx7_interfaces/msg/DetectedObjectArray.msg
  LO CONSUME   robx7_task/robx7_task/task_manager.py a traves de /vision/objects

Salidas
  /vision/objects      lista de objetos con clase y pose
  /tf                  un marco por objeto llamado obj_ seguido del id
  /vision/markers      marcadores para RViz
  /vision/image_debug  imagen con contornos, clase, coordenadas y verdad de terreno

La imagen de depuracion se publica siempre que llegue una imagen. Si falta camera_info o TF
la propia imagen muestra un aviso.
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robx7_interfaces.msg import DetectedObject, DetectedObjectArray
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# LLAMADA A ARCHIVO robx7_vision/robx7_vision/ai_classifier.py
from robx7_vision import ai_classifier as ai
# LLAMADA A ARCHIVO robx7_vision/robx7_vision/detector_core.py
from robx7_vision import detector_core as core

# Color con el que se dibuja cada clase en RViz, componentes r g b entre 0 y 1
RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.1), 'blue': (0.2, 0.3, 1.0)}


def put_text(img, text, org, scale=0.55):
    """Texto blanco con borde negro, se lee sobre cualquier fondo."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Parametros. Se cambian desde el launch o con la opcion de parametros de ros2 run
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        # Se usa este marco y no el frame_id del mensaje para no depender de Gazebo
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('classify_by', 'color')          # variante A color, variante B shape
        self.declare_parameter('classifier', 'classic')         # classic usa HSV, ai usa la red
        self.declare_parameter('model_path', '')                # vacio usa el modelo instalado
        self.declare_parameter('object_height', 0.04)           # altura de los objetos en metros
        self.declare_parameter('min_area', 150.0)
        self.declare_parameter('max_area', 3000.0)
        self.declare_parameter('circularity_threshold', 0.82)
        # Zona de entrada en world. Lo que quede fuera se ignora, por ejemplo los destinos
        self.declare_parameter('roi_x_min', 0.50)
        self.declare_parameter('roi_x_max', 0.90)
        self.declare_parameter('roi_y_min', -0.20)
        self.declare_parameter('roi_y_max', 0.20)
        self.declare_parameter('ground_truth_file', '')
        # Respaldo si camera_info no llega. Campo de vision en radianes, 0.0 lo desactiva
        self.declare_parameter('fallback_hfov', 0.0)
        # Control de carga. Procesar menos cuadros y publicar menos imagenes ahorra CPU y red
        self.declare_parameter('process_rate', 5.0)     # cuadros por segundo que se procesan, 0 procesa todos
        self.declare_parameter('debug_image', True)     # publicar la imagen de depuracion
        self.declare_parameter('debug_rate', 5.0)       # frecuencia maxima de la imagen de depuracion
        self.declare_parameter('debug_scale', 0.5)      # factor de reduccion de la imagen de depuracion
        self.declare_parameter('report_period', 0.0)    # segundos entre informes de error, 0 los apaga

        gp = self.get_parameter
        self.image_topic = gp('image_topic').value
        self.info_topic = gp('info_topic').value
        self.camera_frame = gp('camera_frame').value
        self.target_frame = gp('target_frame').value
        self.classify_by = gp('classify_by').value
        self.classifier = gp('classifier').value
        self.obj_h = gp('object_height').value
        self.min_area = gp('min_area').value
        self.max_area = gp('max_area').value
        self.circ_thr = gp('circularity_threshold').value
        self.roi = (gp('roi_x_min').value, gp('roi_x_max').value,
                    gp('roi_y_min').value, gp('roi_y_max').value)
        self.fallback_hfov = gp('fallback_hfov').value
        self.min_period = 1.0 / gp('process_rate').value if gp('process_rate').value > 0 else 0.0
        self.debug_image = gp('debug_image').value
        self.debug_period = 1.0 / gp('debug_rate').value if gp('debug_rate').value > 0 else 0.0
        self.debug_scale = gp('debug_scale').value
        self.report_period = gp('report_period').value
        self.ground_truth = self.load_ground_truth(gp('ground_truth_file').value)
        self.model = self.load_model(gp('model_path').value)

        # Estado interno
        self.bridge = CvBridge()        # convierte entre sensor_msgs/Image y arreglos de OpenCV
        self.K = None                   # matriz intrinseca, llega en camera_info
        self.n_images = 0               # contadores para el diagnostico
        self.n_info = 0
        self.tf_ok = False
        self.n_objects = 0
        self.last_report = 0.0
        self.vision_ms = 0.0            # promedio movil del tiempo de la etapa de vision
        self.last_processed = -1.0      # instante del ultimo cuadro procesado segun el sello de tiempo
        self.last_debug = -1.0          # instante de la ultima imagen de depuracion publicada

        # TF2. El buffer guarda las transformadas que escucha el listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Entradas y salidas
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image,
                                 qos_profile_sensor_data)
        self.pub_objects = self.create_publisher(DetectedObjectArray, '/vision/objects', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/vision/markers', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/image_debug', 1)

        # Cada 5 s informa el estado y avisa que falta si algo no llega
        self.create_timer(5.0, self.watchdog)

        self.get_logger().info(
            f'Detector listo. Metodo {self.classifier}. Clasificacion por {self.classify_by}. '
            f'Imagen en {self.image_topic}, camera_info en {self.info_topic}')

    # ------------------------------------------------------------------ utilidades
    def load_ground_truth(self, path):
        if not path:
            # LECTURA DE ARCHIVO robx7_vision/config/ground_truth.yaml
            path = os.path.join(get_package_share_directory('robx7_vision'),
                                'config', 'ground_truth.yaml')
        try:
            with open(path) as f:
                return yaml.safe_load(f)['objects']
        except (OSError, KeyError, yaml.YAMLError) as e:
            self.get_logger().warn(f'Sin verdad de terreno ({e})')
            return []

    def load_model(self, path):
        """Carga la red neuronal si se pidio el metodo ai. Si falla usa el metodo clasico."""
        if self.classifier != 'ai':
            return None
        if not path:
            # LECTURA DE ARCHIVO robx7_vision/models/mlp_color_shape.npz
            path = os.path.join(get_package_share_directory('robx7_vision'),
                                'models', 'mlp_color_shape.npz')
        try:
            model = ai.CropClassifier.load(path)
            self.get_logger().info(f'Modelo cargado desde {path}')
            return model
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().error(f'No se pudo cargar el modelo ({e}). Se usa el metodo clasico')
            self.classifier = 'classic'
            return None

    def in_roi(self, p):
        return self.roi[0] <= p[0] <= self.roi[1] and self.roi[2] <= p[1] <= self.roi[3]

    def on_info(self, msg: CameraInfo):
        self.n_info += 1
        if self.K is None or self.n_info == 1:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f'camera_info recibido fx {self.K[0, 0]:.1f} fy {self.K[1, 1]:.1f} '
                f'cx {self.K[0, 2]:.1f} cy {self.K[1, 2]:.1f}')

    def watchdog(self):
        """Mensajes de diagnostico claros cuando algo no llega."""
        log = self.get_logger()
        if self.n_images == 0:
            log.warn(f'NO llegan imagenes en {self.image_topic}. Revisa el puente y la camara con '
                     f'ros2 topic hz {self.image_topic}')
        elif self.n_info == 0 and self.fallback_hfov <= 0.0:
            log.warn(f'Llegan imagenes pero NO camera_info en {self.info_topic}. '
                     f'Revisa con ign topic -l y ros2 topic list. Solucion rapida fallback_hfov 1.2')
        elif not self.tf_ok:
            log.warn(f'Falta la transformada {self.target_frame} hacia {self.camera_frame}. '
                     f'Revisa con ros2 run tf2_ros tf2_echo {self.target_frame} {self.camera_frame}')
        # Si todo esta bien no se imprime nada para no llenar la terminal

    def publish_debug(self, frame, header, lines):
        """Publica la imagen de depuracion si alguien la escucha y respetando su frecuencia maxima."""
        if not self.debug_image or self.pub_debug.get_subscription_count() == 0:
            return
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if 0 <= self.last_debug and 0 <= stamp - self.last_debug < self.debug_period:
            return
        self.last_debug = stamp
        for i, line in enumerate(lines):
            put_text(frame, line, (10, 22 + 20 * i))
        if self.debug_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.debug_scale, fy=self.debug_scale,
                               interpolation=cv2.INTER_AREA)
        img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img.header = header
        self.pub_debug.publish(img)

    # ------------------------------------------------------------------ vision
    def find_blobs_classic(self, frame):
        """Metodo clasico. Una mascara HSV por color y la forma se decide con la circularidad."""
        # LLAMADA A ARCHIVO robx7_vision/robx7_vision/detector_core.py funcion find_blobs
        return core.find_blobs(frame, self.min_area, self.max_area)

    def find_blobs_ai(self, frame):
        """Metodo con red neuronal. Candidatos de cualquier color y la red decide color y forma."""
        # LLAMADA A ARCHIVO robx7_vision/robx7_vision/detector_core.py funcion find_blobs
        candidates = core.find_blobs(frame, self.min_area, self.max_area, core.ANY_COLOR_RANGES)
        for b in candidates:
            # LLAMADA A ARCHIVO robx7_vision/robx7_vision/ai_classifier.py
            crop = ai.crop_contour(frame, b.contour)
            b.color, b.shape, _ = self.model.predict(crop)
        return candidates

    # ------------------------------------------------------------------ callback principal
    def on_image(self, msg: Image):
        self.n_images += 1
        # Se descartan cuadros antes de convertirlos. Es el ahorro mas grande porque
        # convertir una imagen copia casi un megabyte de memoria
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (self.min_period > 0.0 and self.last_processed >= 0.0
                and 0.0 <= stamp - self.last_processed < self.min_period):
            return
        self.last_processed = stamp
        # sensor_msgs/Image a arreglo NumPy de forma alto, ancho, 3 en orden B G R
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Sin intrinsecos no se puede calcular nada 3D, lo decimos en la imagen
        if self.K is None and self.fallback_hfov > 0.0:
            self.K = core.intrinsics_from_fov(msg.width, msg.height, self.fallback_hfov)
            self.get_logger().warn('Usando K calculada con fallback_hfov porque no llego camera_info')
        if self.K is None:
            self.publish_debug(frame, msg.header, ['ESPERANDO camera_info'])
            return

        # Pose de la camara en world segun TF2. Time vacio significa la transformada mas reciente
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame, Time())
            self.tf_ok = True
        except TransformException as e:
            self.tf_ok = False
            self.get_logger().warn(f'Sin TF {self.target_frame} hacia {self.camera_frame} {e}',
                                   throttle_duration_sec=2.0)
            self.publish_debug(frame, msg.header, [f'SIN TF {self.target_frame} hacia '
                                                   f'{self.camera_frame}'])
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        R_wc = core.quat_to_matrix(q.x, q.y, q.z, q.w)
        t_wc = np.array([tr.x, tr.y, tr.z])

        # Vision. Se mide el tiempo para comparar el metodo clasico con la red
        t0 = time.perf_counter()
        blobs = self.find_blobs_ai(frame) if self.model is not None else self.find_blobs_classic(frame)
        ms = (time.perf_counter() - t0) * 1000.0
        self.vision_ms = ms if self.vision_ms == 0.0 else 0.9 * self.vision_ms + 0.1 * ms
        debug = frame.copy()

        # Pixel a punto 3D, interseccion del rayo con el plano de la cara superior
        detections = []
        for b in blobs:
            p = core.pixel_to_world(b.cx, b.cy, self.K, R_wc, t_wc, self.obj_h)
            if p is None or not self.in_roi(p):
                cv2.drawContours(debug, [b.contour], -1, (140, 140, 140), 1)   # descartado
                continue
            shape = b.shape or core.classify_shape(b.circularity, self.circ_thr)
            label = b.color if self.classify_by == 'color' else shape
            detections.append((b, p, shape, label))
        detections.sort(key=lambda d: (d[0].color, d[1][1]))      # orden estable
        self.n_objects = len(detections)

        # Mensaje, TF, marcadores y depuracion
        out = DetectedObjectArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        markers = MarkerArray()
        tfs = []
        counts = {}
        for i, (b, p, shape, label) in enumerate(detections):
            k = counts.get(b.color, 0)
            counts[b.color] = k + 1
            obj_id = f'{b.color}_{k}'

            o = DetectedObject()
            o.id, o.label, o.color, o.shape = obj_id, label, b.color, shape
            o.area_px = float(b.area)
            o.pose.position.x, o.pose.position.y = float(p[0]), float(p[1])
            o.pose.position.z = self.obj_h / 2.0            # centro del objeto
            o.pose.orientation.w = 1.0                      # no se estima la orientacion
            out.objects.append(o)

            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = f'obj_{obj_id}'
            t.transform.translation.x = o.pose.position.x
            t.transform.translation.y = o.pose.position.y
            t.transform.translation.z = o.pose.position.z
            t.transform.rotation.w = 1.0
            tfs.append(t)

            markers.markers.extend(self.make_markers(i, o, out.header))

            cv2.drawContours(debug, [b.contour], -1, (255, 255, 255), 2)
            put_text(debug, f'{label} ({p[0]:.3f}, {p[1]:.3f})',
                     (int(b.cx) - 55, int(b.cy) - 22), 0.5)

        self.pub_objects.publish(out)
        if tfs:
            self.tf_broadcaster.sendTransform(tfs)
        self.pub_markers.publish(markers)

        # Verdad de terreno. Se proyecta al pixel y se mide el error
        self.draw_ground_truth(debug, R_wc, t_wc)
        self.report_error(detections)

        self.publish_debug(debug, msg.header, [
            f'{len(detections)} objeto(s). Metodo {self.classifier}. Clase por {self.classify_by}',
            f'vision {self.vision_ms:.2f} ms. Cruz magenta es la posicion real'])

    # ------------------------------------------------------------------ salidas
    def make_markers(self, idx, o, header):
        r, g, b = RGB[o.color]
        life = Duration(seconds=0.8).to_msg()

        body = Marker()
        body.header = header
        body.ns, body.id = 'objects', idx
        body.type = Marker.CYLINDER if o.shape == 'cylinder' else Marker.CUBE
        body.action = Marker.ADD
        body.pose = o.pose
        body.scale.x = body.scale.y = 0.04
        body.scale.z = 0.05 if o.shape == 'cylinder' else 0.04
        body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.8
        body.lifetime = life

        text = Marker()
        text.header = header
        text.ns, text.id = 'labels', idx
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = o.pose.position.x
        text.pose.position.y = o.pose.position.y
        text.pose.position.z = 0.10
        text.pose.orientation.w = 1.0
        text.scale.z = 0.035
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{o.label}\n({o.pose.position.x:.3f}, {o.pose.position.y:.3f})'
        text.lifetime = life
        return [body, text]

    def draw_ground_truth(self, img, R_wc, t_wc):
        for gt in self.ground_truth:
            px = core.world_to_pixel([gt['x'], gt['y'], self.obj_h], self.K, R_wc, t_wc)
            if px is not None:
                u, v = int(round(px[0])), int(round(px[1]))
                cv2.drawMarker(img, (u, v), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 1)

    def report_error(self, detections):
        """Informa el error de posicion en mm y cuantas etiquetas son correctas. Apagado por defecto."""
        now = time.monotonic()
        if self.report_period <= 0.0 or now - self.last_report < self.report_period or not detections:
            return
        self.last_report = now
        errs = []
        right = 0
        for b, p, shape, _ in detections:
            if not self.ground_truth:
                break
            nearest = min(self.ground_truth, key=lambda g: math.hypot(p[0] - g['x'], p[1] - g['y']))
            errs.append((b.color, math.hypot(p[0] - nearest['x'], p[1] - nearest['y']) * 1000.0))
            right += int(nearest['color'] == b.color and nearest['shape'] == shape)
        if errs:
            txt = ', '.join(f'{c} {e:.1f} mm' for c, e in errs)
            mean = sum(e for _, e in errs) / len(errs)
            self.get_logger().info(
                f'{len(detections)} objeto(s). Error {txt}. Media {mean:.1f} mm. '
                f'Clases correctas {right} de {len(errs)}. Metodo {self.classifier}')


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

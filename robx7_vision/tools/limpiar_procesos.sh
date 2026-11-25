#!/usr/bin/env bash
# Cierra los procesos de Gazebo y de ROS 2 que hayan quedado de ejecuciones anteriores.
# Uso
#   bash limpiar_procesos.sh
# Pide confirmacion antes de cerrar nada. Despues reinicia el demonio de ROS 2.

PATTERNS=("ign gazebo" "gz sim" "ruby" "rviz2" "parameter_bridge" "robot_state_publisher" "controller_manager" "ros2 launch" "object_detector" "task_manager" "camera_viewer" "ros_gz_sim")

echo "Procesos que se cerraran"
for P in "${PATTERNS[@]}"; do
  pgrep -af "$P" | grep -v "limpiar_procesos" | sed 's/^/    /'
done
read -r -p "Cerrar estos procesos? Escribe s para continuar " ANS
if [ "$ANS" != "s" ]; then echo "No se cerro nada."; exit 0; fi

for P in "${PATTERNS[@]}"; do pkill -f "$P" 2>/dev/null; done
sleep 2
for P in "${PATTERNS[@]}"; do pkill -9 -f "$P" 2>/dev/null; done
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
echo "Listo. Ya puedes lanzar de nuevo."

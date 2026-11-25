#!/usr/bin/env bash
# Revisa el estado del equipo. No modifica nada.
# Uso
#   bash diagnostico_equipo.sh
# Se puede ejecutar con Gazebo cerrado, y sirve antes de lanzar o cuando algo falla.

warn() { echo "  [AVISO] $1"; }
ok()   { echo "  [OK]    $1"; }

echo "1) Procesos que pueden haber quedado de ejecuciones anteriores"
PROCS=$(ps -eo pid,etimes,pcpu,comm,args | grep -E "ign gazebo|gz sim|ruby|rviz2|parameter_bridge|robot_state_publisher|controller_manager|ros2 launch|object_detector|task_manager" | grep -v grep)
if [ -z "$PROCS" ]; then ok "No hay procesos de ROS ni de Gazebo corriendo"
else
  echo "$PROCS" | awk '{printf "    pid %s edad %s s cpu %s%% %s\n", $1, $2, $3, $4}'
  N=$(echo "$PROCS" | grep -cE "ign gazebo|gz sim")
  if [ "$N" -gt 2 ]; then warn "Hay $N procesos de Gazebo. Debe haber como maximo dos, servidor e interfaz. Ejecuta limpiar_procesos.sh"; fi
  OLD=$(echo "$PROCS" | awk '$2 > 3600' | wc -l)
  if [ "$OLD" -gt 0 ]; then warn "Hay $OLD procesos con mas de una hora de edad, probablemente de una ejecucion anterior"; fi
fi

echo "2) Configuracion de red de ROS 2"
echo "    ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 por defecto}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-no definido}  RMW=${RMW_IMPLEMENTATION:-por defecto}"
if [ -z "$ROS_DOMAIN_ID" ] && [ "$ROS_LOCALHOST_ONLY" != "1" ]; then
  warn "Usas el dominio 0 y ROS ve la red completa. Si hay otros equipos con ROS 2 en la misma red, sus nodos y su /clock se mezclan con los tuyos. Prueba export ROS_LOCALHOST_ONLY=1"
else ok "El dominio o el modo local estan configurados"; fi

echo "3) CPU y memoria"
NCPU=$(nproc); LOAD=$(cut -d' ' -f1 /proc/loadavg)
echo "    Nucleos $NCPU, carga de 1 minuto $LOAD"
awk -v l="$LOAD" -v n="$NCPU" 'BEGIN{ if (l > n) exit 1 }' || warn "La carga supera el numero de nucleos. El equipo esta saturado"
free -m | awk 'NR==2{printf "    Memoria libre %s MB de %s MB\n", $7, $2} NR==3{printf "    Swap usada %s MB\n", $3}'
SW=$(free -m | awk 'NR==3{print $3}')
if [ "${SW:-0}" -gt 500 ]; then warn "Se esta usando mucha swap. Cierra aplicaciones, la simulacion se vuelve lenta"; fi

echo "4) Energia y frecuencia de CPU"
GOV=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
if [ -r "$GOV" ]; then
  G=$(cat $GOV); echo "    Gobernador de CPU $G"
  if [ "$G" = "powersave" ]; then warn "El gobernador es powersave. Con corriente y para simular puedes usar performance"; fi
fi
for B in /sys/class/power_supply/BAT*; do
  if [ -r "$B/status" ]; then
    S=$(cat $B/status); echo "    Bateria $S"
    if [ "$S" = "Discharging" ]; then warn "El equipo funciona con bateria. Muchos portatiles reducen el rendimiento asi"; fi
  fi
done

echo "5) Render de la camara"
if command -v glxinfo >/dev/null 2>&1; then
  R=$(glxinfo -B 2>/dev/null | grep -i "OpenGL renderer")
  echo "    $R"
  if echo "$R" | grep -qiE "llvmpipe|software"; then warn "El render es por software. La camara de Gazebo consume mucha CPU. Revisa los controladores de la tarjeta grafica"; fi
else echo "    glxinfo no esta instalado, sudo apt install mesa-utils para ver el render"; fi

echo "6) Buffers de red para DDS"
RM=$(sysctl -n net.core.rmem_max 2>/dev/null)
echo "    net.core.rmem_max=$RM"
if [ -n "$RM" ] && [ "$RM" -lt 2097152 ]; then warn "El buffer es pequeno para imagenes grandes. Si ves perdida de mensajes, sudo sysctl -w net.core.rmem_max=2147483647"; fi
echo "Listo."

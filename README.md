# F1TENTH Reactive Racer - Budapest Track

**Autor:** Andrew Emmanuel Cornejo Ramirez
**Pistas de Evaluación:** Budapest.

Este repositorio contiene la implementación de un controlador reactivo para la competencia autónoma F1TENTH en ROS 2. El objetivo principal es completar 10 vueltas consecutivas sin colisiones en el menor tiempo posible.

<div align="center">
  <img src="img/map_Budapest.png" alt="Mapa de Budapest">
</div>

## 📘 Descripción del Enfoque Utilizado
El algoritmo principal ha evolucionado de un simple **Follow the Gap** a una arquitectura de carreras avanzada con **Percepción Dividida (Split-Perception)** y Control PD Dinámico. Al ser puramente reactivo, no depende de mapas globales, procesando la toma de decisiones en tiempo real a través de los siguientes pasos:
1. **Limpieza del LiDAR:** Filtra valores infinitos o nulos del escáner láser, limitando la visión útil a un máximo de 30 metros para evitar falsos positivos en rectas largas.
2. **Burbuja de Seguridad:** Detecta el obstáculo inminente más cercano e "infla" su tamaño forzando a cero los rayos láser adyacentes (`bubble_size = 3`), obligando al algoritmo a trazar trayectorias que protegen el chasis del vehículo.
3. **Identificación de Brechas:** Localiza la secuencia más amplia de espacio libre (el "Gap") para determinar la dirección general de escape.
4. **Percepción Dividida (Split-Perception):** El robot separa su atención en dos objetivos distintos:

* **Visión Periférica (Velocidad):** Evalúa la profundidad del hueco (Gap) para mantener el acelerador a fondo (hasta 7.0 m/s), reduciendo la velocidad de forma exponencial solo cuando el objetivo está a menos de 4.4 metros.

* **Visión Frontal (Dirección):** Un escáner central estrecho (0.40 radianes) vigila exclusivamente la distancia hacia el muro frontal para calcular el momento exacto de giro.
5. **Control PD Dinámico Exponencial:** Utiliza un controlador Proporcional-Derivativo para la dirección. Las ganancias _Kp_ y _Kd_ se mantienen atenuadas en rectas para brindar estabilidad, y se liberan mediante una curva matemática exponencial normalizada solo cuando el muro frontal cruza el umbral crítico de 2.7 metros. Esto fuerza al vehículo a ejecutar un giro de carreras estilo "Late Apex" (Vértice tardío).

## 📂 Estructura del Código
El núcleo del proyecto reside en dos único script optimizados:
* `reactive_follower.py`: Nodo de ROS 2 que se suscribe al tópico `/scan`, procesa la matriz de distancias y publica comandos de tipo `AckermannDriveStamped` en el tópico `/drive`. Contiene un "Panel de Control y Tuning" con parámetros de percepción, seguridad, cinemática de frenado exponencial y agresividad del volante.
* `lap_timer.py`: Juez de carrera autónomo que certifica las 10 vueltas requeridas. Se suscribe a la odometría (`/ego_racecar/odom`) utilizando el perfil `qos_profile_sensor_data` para garantizar la conexión. Este script extrae la marca de tiempo nativa del motor de físicas del simulador (`msg.header.stamp`), autocalibra la línea de meta dinámicamente e implementa un margen de seguridad de 5.0 metros para evitar falsos conteos de vueltas.

## 🚀 Instrucciones de Ejecución
Para compilar y ejecutar este controlador en el simulador oficial de F1TENTH, sigue estos pasos:

**1. Compilar el paquete (Terminal 1)**
```bash
cd ~/F1Tenth_ws
colcon build --packages-select f1tenth_reactive_racer
source install/setup.bash
```

**2. Ejecutar el Simulador (Terminal 1)**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth_ws
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
<img src="img/sim_inicialized.png" width="75%" alt="Simulador Inicializado">

**3. Posicionar el Vehículo y Ejecutar el Juez de Carrera (Terminal 2)**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth_ws
source install/setup.bash

# Teletransporte a la línea de salida
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -12.311, y: 9.941, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.330, w: 0.944}}}}"

# Iniciar el cronometro y contador de vueltas
ros2 run f1tenth_reactive_racer lap_timer_node
```
<img src="img/pos_and_clock_inicialized.png" width="75%" alt="Posición y Reloj Inicializado">

**4. Ejecutar el Controlador (Terminal 3)**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth_ws
source install/setup.bash

# Iniciar el algoritmo Follow the Gap
ros2 run f1tenth_reactive_racer reactive_node
```

<img src="img/control_inizialized.png" width="75%" alt="Control Inicializado">


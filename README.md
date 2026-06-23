# F1TENTH Reactive Racer - Budapest Track

**Autor:** Andrew Emmanuel Cornejo Ramirez
**Pistas de Evaluación:** Budapest (y evaluación con obstáculos dinámicos/estáticos).

Este repositorio contiene la implementación de un controlador reactivo para la competencia autónoma F1TENTH en ROS 2. 

## 📘 Descripción del Enfoque Utilizado
El algoritmo principal se basa en la técnica **Follow the Gap (Disparity Extender)**. El controlador es puramente reactivo, lo que significa que no depende de mapas globales o algoritmos de ruteo como A*. Su proceso de toma de decisiones en tiempo real consta de:
1. **Limpieza del LiDAR:** Filtra valores infinitos o nulos del escáner láser.
2. **Burbuja de Seguridad:** Detecta el obstáculo más inminente e "infla" su tamaño forzando a cero los rayos láser adyacentes, protegiendo así el ancho físico del chasis del vehículo.
3. **Identificación de Brechas:** Localiza la secuencia más amplia de espacio libre (el "Gap").
4. **Control de Dirección y Velocidad:** Utiliza un controlador Proporcional para apuntar la dirección hacia el centro del Gap, calculando un perfil de velocidad dinámico basado en la profundidad de la brecha.

## 📂 Estructura del Código
El núcleo del proyecto reside en dos único script optimizados:
* `reactive_follower.py`: Nodo de ROS 2 que se suscribe al tópico `/scan`, procesa la matriz de distancias y publica comandos de tipo `AckermannDriveStamped` en el tópico `/drive`. Contiene parámetros de sintonización agrupados (Tuning Panel) para ajustar el ángulo de visión, la velocidad cinemática y la agresividad del volante (Kp).
* `lap_timer.py`: Nodo juez de carrera que se suscribe al tópico `/odom` para rastrear la posición del vehículo, detectando el inicio y fin de cada vuelta para contabilizar el total y registrar los tiempos en segundos.

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

**3. Posicionar el Vehículo y Ejecutar el Controlador (Terminal 2)**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth_ws
source install/setup.bash

# Teletransporte a la línea de salida
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -12.311, y: 9.941, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.330, w: 0.944}}}}"

# Iniciar el algoritmo Follow the Gap
ros2 run f1tenth_reactive_racer reactive_node
```

**4. Ejecutar el Juez de Carrera (Terminal 3)**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth_ws
source install/setup.bash

# Iniciar el cronometro y contador de vueltas
ros2 run f1tenth_reactive_racer lap_timer_node
```


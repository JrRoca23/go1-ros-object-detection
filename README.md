Repositorio del TFG sobre detección visual y movimiento del robot Go1 usando ROS
==============================
GO1 - VISION + MOVIMIENTO ROS
==============================

Este documento describe los pasos para lanzar el sistema de visión y control del robot GO1 usando una Raspberry Pi y una cámara ASUS Xtion Pro Live.

----------------------------------------
1. CONECTARSE A LA RASPBERRY POR SSH
----------------------------------------

ssh <tu_usuario>@<IP_RASPBERRY>

(Reemplazar <tu_usuario> y <IP_RASPBERRY> con tus datos, después de haber configurado la raspi y clonado el repositorio)

Asegúrate de que los archivos .bashrc estén configurados con:
- ROS_MASTER_URI
- ROS_IP

----------------------------------------
2. VISIÓN - DETECCIÓN DE BOTELLAS
----------------------------------------

(Desde la Raspberry Pi)

Paso 1: Lanzar la cámara RGB-D

    roslaunch openni2_camera openni2.launch

Paso 2: Lanzar el nodo de detección

    rosrun go1_vision bottle_detector.py

Este nodo:
- Detecta botellas azules.
- Calcula la distancia usando la imagen de profundidad.
- Publica:
    /bottle_position        → Coordenadas de la botella y distancia
    /bottle_detector/image  → Imagen con anotaciones

Paso 3: Visualizar desde tu PC con ROS

    rqt_image_view

Seleccionar el topic:

    /bottle_detector/image

----------------------------------------
3. MOVIMIENTO - TELEOPERACIÓN DEL GO1
----------------------------------------

(Desde la Raspberry o una terminal ROS conectada al robot)

Paso 1: Ejecutar en una terminal:

    rosrun go1_movement highcmd_publisher.cpp

Paso 2: Ejecutar en otra terminal:

    rosrun go1_movement go1_teleop.cpp

(Esto permite controlar el Go1 con el teclado)

* Pendiente: Crear un launch para lanzar los dos nodos juntos *

----------------------------------------
4. MOVIMIENTO AUTÓNOMO HACIA BOTELLAS
----------------------------------------

(Pendiente de implementar)

Objetivo:
- Si la botella se detecta a más de 2.0 metros → no moverse.
- Si está entre 2.0 y 0.5 metros → avanzar hacia ella.
- Si está a menos de 0.5 metros → detenerse.

Este nodo deberá suscribirse a:

    /bottle_position

Y publicar comandos de movimiento al Go1 automáticamente.

----------------------------------------
ESTADO ACTUAL
----------------------------------------

- Visión: funcionando
- Detección de botellas: funcionando
- Estimación de distancia: funcionando
- Movimiento teleoperado: funcionando
- Movimiento autónomo: PENDIENTE
- Launch de teleop: PENDIENTE

----------------------------------------
NOTAS FINALES
----------------------------------------

- La detección funciona bien incluso a 2 metros o más.
- Ajustes de umbral de color HSV y área mínima permiten mejorar la precisión.
- Asegúrate de tener buena iluminación para mejor rendimiento.

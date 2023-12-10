# TFG: Creación de Gemelos Digitales para la Simulación Robótica. 
# Rama: problemas_robot
Rama dedicada a la visualización de la simulación dentro de un laberinto, pero únicamente controlado por el controlador nativo de ROS2: 

En la primera terminal para iniciar la simulación con el robot y el laberinto:

```bash
ros2 launch robot mundo.launch.py
```

En una terminal paralela, correr el siguiente código para iniciar ejecutar el programa que controla el robot con el teclado:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
[Vídeo explicativo](https://drive.google.com/file/d/1YqFXenKSTBmX3yUfVSwQUJPj2Xmp33Rm/view?usp=drive_link): 

![](robot_controlado_teclado.gif)
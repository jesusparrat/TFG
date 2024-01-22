# TFG: Creación de Gemelos Digitales para la Simulación Robótica. 
## Rama: FINAL
Rama dedicada a subir los archivos finales del proyecto: Creación de Gemelos Digitales para la Simulación Robótica. El robot identificado para el proyecto será un pequeño robot que cumpla las condiciones de la competición Micromouse, y desarrollaremos la simulación en el entorno digital de Gazebo, apoyados por Robotics Operating System 2 (ROS 2). 

### Utilización del repositorio:
Descargar la carpeta 
```bash
/src
``` 
en cualquier carpeta local del PC. Convertirla en workspace de ROS 2 ejecutando dentro de esa carpeta, en una terminal:
```bash
cd /carpeta_base
colcon build
```
Una vez aquí, solo falta hacer el source al setup:
```bash
cd /carpeta_base
source install/setup.bash
``` 
#### Ejecución del proyecto:
Inicializar el mundo primero:
```bash
ros2 launch final mundo.launch.py
ros2 launch final 1_mundo.launch.py*
ros2 launch final 2_mundo.launch.py**
```
Ejecutar un mundo a la vez. 

\* Es la inicialización del mundo con el robot pero con un laberinto simple.

\** Es la inicialización del mundo con el robot pero con un laberinto más complejo para testear el funcionamiento de los algoritmos. 

En una terminal paralela, con cualquier mundo ejecutándose: 
```bash
ros2 run final maze_solver
```
Después de que se generen las ventanas de la fase de localización, pulsar una tecla para las ventanas de la fase de mapeo. Otra tecla más para las rutas, y al final se ejecuta el nodo de resolución de laberintos. 




Video Demo del laberinto original: 
<video width="640" height="420" controls>
  <source src="final_laberinto_O.webm" type="video/webm">
</video>

Video Demo del laberinto simple
<video width="640" height="420" controls>
  <source src="final_laberinto_1.webm" type="video/webm">
</video>

Video Demo del laberinto complejo
<video width="640" height="420" controls>
  <source src="final_laberinto_2.webm" type="video/webm">
</video>


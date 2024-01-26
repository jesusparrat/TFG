# TFG: Creación de Gemelos Digitales para la Simulación Robótica.

## Rama: FINAL

Rama dedicada a subir los archivos finales del proyecto "Creación de Gemelos Digitales para la Simulación Robótica". El robot seleccionado para el proyecto es un pequeño robot que cumple con las condiciones de la competición Micromouse. La simulación se desarrollará en el entorno digital de Gazebo, con el respaldo de Robotics Operating System 2 (ROS 2).

### Utilización del repositorio:

Descargar la carpeta `/src` y colocarla en cualquier carpeta local de la PC. Convertirla en un espacio de trabajo de ROS 2 ejecutando el siguiente comando dentro de esa carpeta en una terminal:

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


### Video Demo del laberinto original:
<video width="320" height="240" controls>
  <source src="final_laberinto_O.mp4" type="video/mp4">
</video>

[Video Demo del laberinto original.]((https://drive.google.com/file/d/1qx4PtUpN_my_yLa5Wj6ZzexJsb1gqxBW/view?usp=sharing)) 



### Video Demo del laberinto simple:
<video width="320" height="240" controls>
  <source src="final_laberinto_1.mp4" type="video/mp4">
</video>

[Video Demo del laberinto original.]((https://drive.google.com/file/d/1eDpDY_rgECqPpCkO8eoJQ5hFiiXthry0/view?usp=sharing)) 



### Video Demo del laberinto complejo:

<video width="320" height="240" controls>
  <source src="final_laberinto_2.mp4" type="video/mp4">
</video>

[Video Demo del laberinto original.]((https://drive.google.com/file/d/1uCoZC0gMX9zqMhfWEvaYSOzOpYkl1dRW/view?usp=sharing)) 

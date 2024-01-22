# TFG: Creación de Gemelos Digitales para la Simulación Robótica. 
## Rama: FINAL
Rama dedicada a subir los archivos finales del proyecto 

### Utilización:
Descargar la carpeta 
```bash
/src
``` 
en cualquier carpeta local del PC. Convertirla en workspace de ROS 2:
```bash
cd /carpeta_base
colcon build
```
Una vez aquí, solo falta hacer el source al setup:
```bash
cd /carpeta_base
source install/setup.bash
``` 
#### Ejecución
```bash
ros2 launch final mundo.launch.py
```
En una terminal paralela: 
```bash
ros2 run final maze_solver
```
Después de que se generen las ventanas de la fase de localización, pulsar una tecla para las ventanas de la fase de mapeo. Otra tecla más para las rutas, y al final se ejecuta el nodo de resolución de laberintos. 

[Video Demo](h[](url)ttps://drive.google.com/file/d/1a72q6izeQrHVVZc0xoRs60AVSFdOi1KO/view?usp=sharing): 

![](demo.gif)

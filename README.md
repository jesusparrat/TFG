# TFG: Creación de Gemelos Digitales para la Simulación Robótica. 
## Rama: problemas_robot
Rama dedicada a resolver los problemas que me surgieron con el robot: le mandaba mensajes para que se moviera de forma lineal y giraba, y le mandaba mensajes de forma angular y se movía recto. 

### Problemas: 
Se puede obtener la info de la velocidad del robot con:  
```bash
ros2 topic info /cmd_vel
``` 
que manda mensajes Twist. Entonces, se pueden ver los mensajes Twist que son intrínsecos del robot con: 
```bash
ros2 interface show geometry_msgs/msg/Twist
```
Y obviamente tenemos un vector 3D de velocidad linear con x, y, z, y también otro vector 3D de velocidad angular x, y, z.

Se crea entonces un programa en Python que publica mensajes en Twist, hago le mando un mensaje 
```bash
msg.linear.x = 0.5
``` 
y el robot no se mueve en dirección lineal sino en angular, y viceversa mandando: 
```bash
msg.angular.x = 0.5
```
el robot se mueve en línea recta, en lugar de en forma angular. 

## Solución: 
Cambiar el modelo del robot de nuevo, ya que el que creé daba fallos. Este nuevo robot se encuentra en la Rama: [robot_laberinto_keyboard](https://github.com/jesusparrat/TFG/tree/robot_laberinto_keyboard). Las modificaciones fueron: cambiar el archivo 
```text
src/
└── robot/
    └── meshes/
        └── base_link.stl

```
directamente desde el origen: creando otro diferente. Así, también se escalaron los archivos 
```text
src/
└── robot/
    └── meshes/
        ├── front_caster.stl
        ├── wheel_l.stl
        └── wheel_r.stl
```
para hacerlos acorde al nuevo archivo de la base, resultando en: 
```text
src/
└── robot/
    └── meshes/
        ├── base_link.stl
        ├── front_caster_PR.stl
        ├── wheel_l_PR.stl
        └── wheel_r_PR.stl
```
que está ubicada en la rama siguiente. 
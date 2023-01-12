# Race-Car
ROS package for an Autonomous Mobile Robot using the ackermann steering geometry

-----------------------------------------------------------------------------------------------------------------------------------------------------------
## Dependencias
```bash
  sudo apt install ros-$ROS_DISTRO-gazebo-ros-control
  sudo apt install ros-$ROS_DISTRO-effort-controllers
  sudo apt install ros-$ROS_DISTRO-joint-state-controller
  sudo apt install ros-$ROS_DISTRO-driver-base
  sudo apt install ros-$ROS_DISTRO-ackermann-msgs
  sudo apt install ros-$ROS_DISTRO-rtabmap-ros
  sudo apt install ros-$ROS_DISTRO-teb-local-planner

  sudo apt install tcl-dev tk-dev python3-tk
```
  
## Guardar el mapa del mundo
1. Crear un nuevo workspace con el paquete
```bash
    cd ~/racecar_ws/src
    git clone https://github.com/Luperbal/Race-Car.git
    cd ..
    catkin_make
 ```
2. Abrir el mundo en Gazebo
```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    roslaunch racecar_gazebo racecar_runway.launch
```
3. Abrir RVIZ para generar el mapa
```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    roslaunch racecar_gazebo slam_gmapping.launch
```
4. Guardar el mapa
```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    rosrun map_server map_saver -f ~/racecar_ws/src/racecar/racecar_gazebo/map/mapa
```
  
## Navegación
1. Abrir el mundo en Gazebo
```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    roslaunch racecar_gazebo racecar_runway_navigation.launch
 ```
 2. Abrir RVIZ. Haciendo click en 2D Nav Goal y en la posición deseada del mapa, se genera el camino que seguirá el robot
 ```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    roslaunch racecar_gazebo racecar_rviz.launch
 ```
3. Llamar al script de path\_pursuit.py para que el robot siga el camino:
 ```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    rosrun racecar_gazebo path_pursuit.py
 ```

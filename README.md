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
1. Abrir el mundo en Gazebo

```bash
    cd ~/racecar_ws/src
    catkin_make
    source devel/setup.bash
    roslaunch racecar_gazebo racecar_runway.launch
```

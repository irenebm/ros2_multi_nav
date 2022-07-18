# TFG - Exploración de un entorno multirobot con Nav2 

[![GitHub Action
Status](https://github.com/irenebm/ros2_multi_nav/workflows/master/badge.svg)](https://github.com/irenebm/ros2_multi_nav)
[![codecov](https://codecov.io/gh/irenebm/ros2_multi_nav/master/graph/badge.svg)](https://codecov.io/gh/irenebm/ros2_multi_nav)

<p align="center">
<img src="https://github.com/irenebm/bitacora_tfg_multirobot/blob/main/wiki/pngwing.com%20(1).png" width="100"/>
</p>

## Objetivo: mapear un entorno con robots reales que trabajen de manera autónoma y colaborativa.
Para llevar a cabo este objetivo el trabajo constará de las siguientes fases:
* [1 robot simulado teleoperado](https://github.com/irenebm/bitacora_tfg_multirobot/wiki/1---SLAM-1-Robot-simulado-teleoperado)
* [1 robot simulado autónomo](https://github.com/irenebm/bitacora_tfg_multirobot/wiki/2---SLAM-1-robot-simuado-aut%C3%B3nomo)
* [2 robots simulados autónomos](https://github.com/irenebm/bitacora_tfg_multirobot/wiki/3-SLAM-2-robots-simulados-aut%C3%B3nomos)

![](https://github.com/irenebm/bitacora_tfg_multirobot/blob/main/wiki/robotintro_dribble.gif)

## Ejecución:

### 1 robot
```
ros2 launch nav2_bringup multi_tb3_simulation_launch.py slam:=True

ros2 launch multirobot_map_merge map_merge.launch.py 

ros2 run multi_nav2 check_pf 
ros2 run multi_nav2 patrolling_main 
```


### 2 robots
```
ros2 launch nav2_bringup multi_tb3_simulation_launch.py slam:=True

ros2 launch multirobot_map_merge map_merge.launch.py 

ros2 run multi_nav2 check_pf 
ros2 launch multi_nav2 multi_robot_patrol.py
```

### 3 robots
```
ros2 launch nav2_bringup multi_tb3_simulation_launch.py slam:=True

ros2 launch multirobot_map_merge map_merge.launch.py 

ros2 run multi_nav2 check_pf 
ros2 launch multi_nav2 multi_robot_patrol.py
```

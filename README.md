# Control_TurtleSim3
This code is used to control the turtle simulator for go to goal. ROS 2.
Para utilização copie a pasta 'py_pubsub' para 'ros2_ws/src'. Em seguida Direcione-se para o diretório 'ros2_wc/', e utilize os seguintes comandos:
``` shell
source install/setup.bash
colcon build --packages-select py_pubsub
```
Inicialização do rviz2 + gazebo: 
``` shell
ROS_DOMAIN_ID=<inteiro(0 a 100)> ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Inicialização do pacote de controle rudimentar(não conta com desvio de obstáculos):
``` shell
ROS_DOMAIN_ID=<inteiro(0 a 100)> ros2 run py_pubsub control 
```
Após o comando acima o robo irá se mover para a lista de posições goal estabelecidas:
``` shell
self.poses = [[-1.0,1.5],[-2.0,-0.1],[-1.5,-0.7]]
```
Quando a distância euclidiana entre a posição atual e a posição goal for inferior a 0.2, conta que objetivo foi alcançado e o controle atua para a pŕoxima posição goal.
Em caso de colisões com os objetos do turtlebot3_world, utilize a linha de comando para zerar as acelerações após fechar o pacote de controle:
``` shell
ROS_DOMAIN_ID=<inteiro(0 a 100)> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
Demonstração: <br />
<p align="center">
<img src="https://github.com/lorenzoppx/Control_TurtleSim3/blob/main/control.gif" width="300">
<p />

(25.07.26작성)
# Using turtlesim, ros2, and rqt
* 목표: turtlesim 패키자를 사용하고, 다음 튜토리얼을 위해 rqt tools를 사용한다.

## Background
turtlesim은 ros2를 배우기 위한 가벼운 시뮬레이터이다. 나중에 실제로봇이나 로봇시뮬레이터를 돌리기위한 ros2의 가장 기초단계의 아이디어를 준다.   
ros2 도구는 사용자가 ROS 시스템을 관리하고,상호작용하는 방식이다. 시스템과 작동의 다양한 측면을 대상으로 하는 여러 명령어를 지원한다. node를 시작하고, parameter를 설정하고, topic을 listen하는 등 다양한 용도로 사용할 수 있습니다. ROS2 도구는 core ROS 2 설치의 일부이다.    
rqt는 ros2의 graphical user interface (GUI) tool이다. rqt에서 수행되는 모든 작업은 명령줄에서 수행할 수 있지만, rqt는 ROS 2 요소를 조작하는 더 사용자 친화적인 방법을 제공한다.     
이 튜토리얼에서는 upon core ros2 concepte인 nodes, topics, services를 다룬다. 이 모든 개념은 나중 튜토리얼에서 자세히 설명될 것이기에 지금은 도구를 설정하고 느낌만 알면 된다. ㅎ     
## Prerequisites
Configuring environment에서 햇던거를 실행하라
## Tasks
### Install turtlesim
매번 새로운 터미널에서는 setup files의 sourcing을 시작해야한다. turtlesim package를 다운받기 위해서는   
```
$sudo apt update
$sudo apt install ros-humble-turtlesim
```
하지만 ros2를 다운받았다면 같이 깔려있을 것이다. 이 패키지가 다운받아져 잇는지 알아모려면 다음의 명령어를 실행하여 다음과 같이 나오는지 확인해본다. 
```terminal
$ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

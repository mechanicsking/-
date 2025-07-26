(25.07.26작성)
# Using turtlesim, ros2, and rqt
* 목표: turtlesim 패키자를 사용하고, 다음 튜토리얼을 위해 rqt tools를 사용한다.

## Background
turtlesim은 ros2를 배우기 위한 가벼운 시뮬레이터이다. 나중에 실제로봇이나 로봇시뮬레이터를 돌리기위한 ros2의 가장 기초단계의 아이디어를 준다.   
ros2 도구는 사용자가 ROS 시스템을 관리하고,상호작용하는 방식이다. 시스템과 작동의 다양한 측면을 대상으로 하는 여러 명령어를 지원한다. node를 시작하고, parameter를 설정하고, topic을 listen하는 등 다양한 용도로 사용할 수 있습니다. ROS2 도구는 core ROS 2 설치의 일부이다.    
rqt는 ros2의 graphical user interface (GUI) tool이다. rqt에서 수행되는 모든 작업은 명령줄에서 수행할 수 있지만, rqt는 ROS 2 요소를 조작하는 더 사용자 친화적인 방법을 제공한다.     
이 튜토리얼에서는 upon core ros2 concepte인 nodes, topics, services를 다룬다. 이 모든 개념은 나중 튜토리얼에서 자세히 설명될 것이기에 지금은 도구를 설정하고 느낌만 알면 된다. ㅎ     
## Prerequisites
Configuring environment에서 햇던거를 실행해준다. 
## Tasks
### 1. Install turtlesim
매번 새로운 터미널에서는 setup files의 sourcing을 시작해야한다. turtlesim package를 다운받기 위해서는   
```
$sudo apt update
$sudo apt install ros-humble-turtlesim
```
하지만 ros2를 다운받았다면 같이 깔려있을 것이다. 이 패키지가 다운받아져 잇는지 알아모려면 다음의 명령어를 실행하여 다음과 같이 나오는지 확인해본다. 
```
$ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```
### 2. Install turtlesim
turtlesim을 실행하기 위해서 다음의 명령어를 입력한다.    
```
$ros2 run turtlesim turtlesim_node
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
실행하면 다음과 같이 터틀 name과 좌표의 기본값이 나타난다.     
다음 사진과 같이 화면이 나타난다.    

<img width="463" height="476" alt="Screenshot from 2025-07-27 00-15-50" src="https://github.com/user-attachments/assets/d439b515-bbcd-41d1-a399-0d8844080352" />    
### 3. Use turtlesim
터틀심을 움직이기 위해서는 새로운 터미널을 열어준고 ros2를 실행해준다. 그리고 다음의 명령어를 통해 터틀심을 움직여준다.      
```
$ros2 run turtlesim turtle_teleop_key
```
이 시점에서는 3개의 윈도우가 열려있게 된다. turtlesim_node를 작동시키는 터미널과 turtle_teleop_key를 실행시키는 터미널, 그리고 위의 화면과 같은  turtlesim window이다.      
키보드의 키를 통해 터틀을 움직일 수 있다. 터틀은 움직이며 경로를 만들어낸다. 키를 누르면 터틀이 짧은 거리를 이동했다가 멈출 것이다.      
만약 node, 연관된 topic, services, action을 확인해보고싶다면 list subcommand들을 사용하면 된다.     

```
$ros2 node list
$ros2 topic list
$ros2 service list
$ros2 action list
```

<img width="407" height="94" alt="image" src="https://github.com/user-attachments/assets/bf6b4a3d-45e2-4478-9664-b7a606aced00" />       
<img width="399" height="114" alt="image" src="https://github.com/user-attachments/assets/9e4659e0-d5a7-41c7-a915-95f3c870647c" />      
<img width="384" height="370" alt="image" src="https://github.com/user-attachments/assets/35d469e4-aba5-4c7b-aca1-e3594625a078" />      
<img width="289" height="49" alt="image" src="https://github.com/user-attachments/assets/2794407e-8ec6-48f7-84ec-3d3a42aff31b" />      
다음과 같이 잘 실행되는 모습을 확인해 볼 수 있다. 해당 개념은 다음 튜토리얼에서 배우게 된다. 본 챕터에서는 단지 turtlesim이 뭔지만 알아본다.      

### 4. Install rqt
다음과 같이 rqt를 다운받는다. 하지만 ros2를 깔았다면 

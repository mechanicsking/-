# 3장 ROS 기초 명령
## 3-1 turtlesim 실행하기
node란? ros에서 실행 가능한 최소한의 단위

node 실행하는 명령어 

    ~$ ros2 run <PKG Name> <Node Name>
turtlesim_node 실행

    ~$ ros2 run turtlesim turtlesim_node
## 3-2 다시 강조하는 setup.bash
sudo apt install 명령으로 설치한 ros pkg들이 저장된 곳

/opt/ros/humble

해당 경로의 setup.bash를 source로 읽음

따라서 source /opt/ros/humble/setup.bash 라는 명령을 사용

그걸 우리는 .bashrc에 자동으로 실행하게 만들어놨다.
## 3-3 Ros2 Node
    ~$ ros2 node list #실행중인 node의 리스트
    ~$ ros2 node info <Node Name>
## 3-4 Ros2 Service 기초
서비스란? 서비스 서버와 서비스 클라이언트가 있을때 클라이언트가 서버에 요청(request)를 하면 반응(response)해주는 것

    ~$ ros2 service list #서비스 리스트 확인
    ~$ ros2 service type <Service Name> #서비스의 타입 확인
서버와 클라이언트의 서비스 타입이 다를 수 잇다.

service definition과 서비스의 개념

어떤 node가 request하면 다른 node가 response해준다. 이때 이 사이에서 definition이 필요함(type)      
<img width="1197" height="641" alt="image" src="https://github.com/user-attachments/assets/e7eb581a-13d6-4bc3-936a-ee53ea16b24e" />     


뭔가를 요쳥할 때 데이터를 줘야할 때가 있다. 그때 데이터의 타입들이 정의가 되어있어야함. 리스폰스 할때에도 그 데이터의 타입들이 정의가 되어있어야함. 이것들을 하나의 파일에 기록을 해두어야한다. 이를 기록해둔 파일이 .srv이다.
Request와 Response를 구분하기 위해 다음과 같이 표기한다.

    Request
    ----
    Response
따라서 ----위에있는건 request 데이터 타입이고 아래있는 것은 response타입이다.

만약 서비스 definition을 터미널에서 확인하고 싶다면

    ros2 interface show turtlesim/srv/TeleportAbsolute
ros에서의 단위는 kg, sec, m와 각도에서는 rad을 선호
## 3-5 Ros2 Service call 1
service를 call하는 명령 -tab을 적절히 사용하자

    ~$ ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 2, y: 2, theta: 1.57}"
이 서비스를 통해 이동

    ~$ ros2 service type /reset
reset이라는 서비스 확인

    ~$ ros2 service call /reset std_srvs/srv/Empty {}
제자리로 돌려줌
## 3-6 Ros2 Service call 2
    /clear
    /kill
    /reset
    /spawn
    /turtle1/set_pen
    /turtle1/teleport_absolute
    /turtle1/teleport_relative
    /turtlesim/describe_parameters
    /turtlesim/get_parameter_types
    /turtlesim/get_parameters
    /turtlesim/list_parameters
    /turtlesim/set_parameters
    /turtlesim/set_parameters_atomically
spawn에 대해 알아보자

    ros2 service type /spawn
-->turtlesim/srv/Spawn
어떻게 구성되는 지 알아보기
```
    ros2 interface show turtlesim/srv/Spawn

    float32 x
    float32 y
    float32 theta
    string name # Optional.  A unique name will be created and returned if this is empty
    ---
    string name
```

---위에 쪽 입력: x, y, theta, stringname을 받음,

---아래쪽 출력
```
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 1, y: 1, theta: 0, name: ''}"
```
<img width="498" height="538" alt="image" src="https://github.com/user-attachments/assets/099c065e-9579-4281-87e2-40862c187607" />       

그 후 다시 service list를 확인해보면 turtle2가 추가되어있음   
```
~$ ros2 service list
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtle2/set_pen
/turtle2/teleport_absolute
/turtle2/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
다시 reset
```
~$ ros2 service call /reset std_srvs/srv/Empty {}
```
## 3-7 ROS2 Topic 1
<img width="1148" height="665" alt="image" src="https://github.com/user-attachments/assets/b99b70f3-97bf-4bb6-890c-9e934a11f52c" />     
 
Publisher 노드가 topic을 발행하면 듣고 싶은 subscriber노드가 듣는다. 듣는 방법은 토픽의 이름과 메시지 타입을 알면 다 들을 수 있다. 디버그와 확장성에 유리하다!

작동중인 토픽을 확인해보고 싶다면
```
~$ ros2 topic list
~$ ros2 topic list -t
~$ ros2 topic list -v
```
토픽을 듣고 싶다면???
```
~$ ros2 topic echo [토픽 이름]
```
## 3-8 ROS2 Topic 2
ros에서 주행하는 것들은 거의 대부분 cmd_vel 토픽을 사용한다.(commend of velocity)     
그렇다면 cmd_vel 토픽의 데이터 타입인 Twist는 어떻게 생겼는지?
```~$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

```
<img width="1148" height="665" alt="image" src="https://github.com/user-attachments/assets/fd8b903c-f2d4-462b-871e-3fa124bde5ae" />        

topic을 구독하는 것은 ros2 topic echo이고 topic을 쏴주는 것은 ros2 topic pub이다. --once는 한번만이라는 뜻!
```
~$ ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
만약 계속 움직이게 하고 싶다면 --once 를 --rate 1로... 뒤에 1은 Hz이다. 1Hz는 초당 1번      
<img width="1200" height="1187" alt="image" src="https://github.com/user-attachments/assets/ba95663d-9b7c-4f25-9e03-2d26b6d88a01" />       
## 3-9 ROS2 Aciton
ros2에서 실시간스럽게 데이터를 전송하는 방법: service, topic, action     
<img width="1169" height="689" alt="image" src="https://github.com/user-attachments/assets/2491fc92-5d17-4bbd-89e5-a922ccfaecd0" />     

<img width="1162" height="630" alt="image" src="https://github.com/user-attachments/assets/4cf756fb-9500-4308-b413-b53734356e09" />      

aciton은 service와 topic이 섞여있음        
action의 list확인     
```
~$ ros2 action list
~$ ros2 action list -t
```
action을 iterface show하면
```
goal 목표
---
결과
---
피드백
```
action에 goal을 주고 싶다면 send_goal 명령어 사용, 다음의 예시를 확인해보자.     

action을 iterface show하면     
```
~$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}"
```
<img width="837" height="256" alt="image" src="https://github.com/user-attachments/assets/baf45f77-0353-4ee0-b98f-e4ea29863ec4" />     

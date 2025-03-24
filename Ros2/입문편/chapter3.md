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

    ros2 interface show turtlesim/srv/Spawn
-->
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

---위에 쪽은 받는 것
x, y, theta, stringname을 받음

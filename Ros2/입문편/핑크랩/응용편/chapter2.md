## 2.1 토픽 메시지 정의 만들기
메시지 정의 만들고 토픽과 서비스에서 다루기      
먼저 메시지 정의를 위해 새로운 패키지를 만든다. 워크스페이스의 src 폴더에서 진행한다.      
```
ros2 pkg create --build-type ament_cmake my_first_package_msgs
```
c++로 하는이유? -> ament_cmake가 메시지를 빌드해주는 기능이 있다.        
tree를 확인해보면 다음과 같다. 
```
.
├── my_first_package
│   ├── my_first_package
│   │   ├── __init__.py
│   │   ├── my_first_node.py
│   │   ├── my_publisher.py
│   │   ├── my_subscriber.py
│   │   └── __pycache__
│   │       ├── __init__.cpython-310.pyc
│   │       ├── my_first_node.cpython-310.pyc
│   │       ├── my_publisher.cpython-310.pyc
│   │       └── my_subscriber.cpython-310.pyc
│   ├── package.xml
│   ├── resource
│   │   └── my_first_package
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
└── my_first_package_msgs
    ├── CMakeLists.txt
    ├── include
    │   └── my_first_package_msgs
    ├── package.xml
    └── src

9 directories, 17 files
```
my_first_package_msgs를 확인해보면, CMakeLists.txt가 있는 것을 확인할 수 있다. 이제 my_first_package_msgs를 열어서 mkdir msg를 해주고 vscode를 연다. msg폴더에 CmdAndPoseVel.msg파일을 만들어준다. 그리고 그 파일에 다음과 같은 내용을 작성한다. 
```
float32 cmd_vel_linear
float32 cmd_vel_angular

float32 pose_x
float32 pose_y
float32 linear_vel
float32 angular_vel
```
cmd_vel은 직선주행명령과 회전주행명령을 가진 토픽이였다. 그리고 pose토픽은 자신의 x, y, z와 속도를 보여주는 데이터 타입이였다. 따라서 이 데이터 타입은 cmd_vel과 pose토픽에서 내가 필요한 것만 가지고 와서 사용하려는 의도가 있는 데이터 타입이다. 이렇게 저장하고 CMakeLists.txt파일을 연다. 그리고 if전에 다음의 명령어를 추가한다.
```txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdAndPoseVel.msg"
)
```

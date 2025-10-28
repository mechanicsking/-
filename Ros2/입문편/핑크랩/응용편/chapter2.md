## 2.1 토픽 메시지 정의 만들기
메시지 정의 만들고 토픽과 서비스에서 다루기      
int a;와 같이 메시지 정의를 미리 해둔다.     
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
그리고 package.xml파일에도 다음과 같이 추가해준다. 
```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
이제 워크스페이스에 돌아와서 colcon build를 해준다음, 워크스페이스를 소스해준후 내가 만든 메시지 타입이 내 터미널에서 잘 인식이 되는지 보려면 다음과 같이 작성
```
ros2 interface show my_first_package_msgs/msg/CmdAndPoseVel 
```
다음과 같은 출력을 얻을 수 있다. 
```
float32 cmd_vel_linear
float32 cmd_vel_angular

float32 pose_x
float32 pose_y
float32 linear_vel
float32 angular_vel
```
## 2.3 토픽 구독하여 메시지 출력해보기
노드 하나에 두개의 토픽을 구독하고 하나의 토픽을 발행해보자.    
다시 미리 만들어둔 my_first_package로 돌아가서 새로운 node를 만든다. 파일이름은 turtle_cmd_and_pose.py로 짓는다.    
미리 setup.py에 가서 엔트리 포인트를 등록해둔다. 
```python
'turtle_cmd_and_pose = my_first_package.turtle_cmd_and_pose:main'
```
이제 소스코드를 작성해보자.      
먼저 import부분이다. 
```python
import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
```
rclpy를 rp로 해주고 rclpy.node의 Node를 가지고 오고, turtlesim.msg의 Pose만 import 해준다. Pose타입을 사용할 것이기 때문에 import하였다.     
이제 CmdAndPose라는 이름으로 class를 만들어준다.  
```python
class CmdAndPose(Node):
    def __init__(self):
        super().__init__('turtle_cmd_pose')
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback_pose, 10)
        
    def callback_pose(self, msg):
        print(msg)
```
__init__을 해주고 super()을 통해 Node의 __init__을 가지고 오고, turtle_cmd_pose라는 이름으로 node를 만들어젔다. 이 노드의 기능으로 create_subscription을 한개 만들건데 Pose타입의 /turtle1/pose토픽에서 subscripte를 할 것이고 이 토픽이 발행해서 들어올때마다 callback_pose를 실행해준다. 이 callback_pose는 msg를 print하는 함수이다. 그리고 메인문을 작성해준다.     
```python
def main(args=None):
    rp.init(args=args)
    
    turtle_cmd_pose_node = CmdAndPose()
    rp.spin(turtle_cmd_pose_node)
    
    turtle_cmd_pose_node.destroy_node()
    rp.shutdown()
    
if __name__ == '__mian__':
    main()
```
위에서 만든 class를 가지고 와서 turtle_cmd_pose_node에 선언해두고, 이 node를 spin돌린다. 이렇게 노드를 만들고 워크스페이스에 돌아와서 colcon build한다.     
이후 turtlesim_node를 키고, 우리가 만든 turtle_cmd_and_pose 노드를 run해주면 다음과 같이 작동 되는 것을 확인할 수 있다.       
<img width="1350" height="821" alt="image" src="https://github.com/user-attachments/assets/cf82541e-5423-4b00-9f67-c3a5918b65a1" />       

우리는 이제 이 노드에 토픽을 발행하거나 구독하는 것을 더 추가해 볼것이다. 
## 2.4 내가 정의한 메시지 사용해보기
이제 이 노드에 2.1에서 새로 정의한 메시지 정의를 import하도록 코드를 수정해보자.  
```python
from my_first_package_msgs import CmdAndPoseVel
```
그리고 CmdAndPoseVel을 객체화한다. 
```python
self.cmd_pose = CmdAndPoseVel()
```
그리고 turtlesim의 pose 토픽을 구독할 때 실행하는 콜백을 변경한다.    
```python
    def callback_pose(self, msg):
        self.cmd_pose.pose_x = msg.x
        self.cmd_pose.pose_y = msg.y
        self.cmd_pose.linear_vel = msg.linear_velocity
        self.cmd_pose.angular_vel = msg.angular_velocity
        print(self.cmd_pose)
```

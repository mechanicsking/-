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
from my_first_package_msgs.msg import CmdAndPoseVel
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
이제 colcon build를 해야하는데 이번에는 빌드를 하나의 패키지만 콕 찍어서 빌드해보자     
```
colcon build --packages-select my_first_package
```
turtlesim_node를 키고 turtle_cmd_and_pose를 키면 다음과 같이 작동한다.        
<img width="1848" height="809" alt="image" src="https://github.com/user-attachments/assets/aa0cf774-48c8-455c-be72-cc9cd3ac4436" />          

지금까지 우리는 토픽하나를 구독해서 내가 정의한 메시지에 이 값을 넣어 print해보았다.       
## 2.5 하나의 노드에서 두 개 이상의 토픽 다루기 1
우리는 cmd_vel 토픽을 구독하고 이를 저번에 만들어둔 pose토픽과 같이 출력할 것이다.       
먼저 Twist타입을 import해준다.     
```python
from geometry_msgs.msg import Twist```
이제 cmd_vel 토픽 구독을 위해 init 부분을 수정하자.    
```python
self.sub_cmdvel = self.create_subscription(Twist, '/turtle1/cmd_vel', self.callback_cmd, 10)
```
또하나의 토픽을 subscription한다. Twist타입의 /turtle1/cmd_vel 토픽을 구독할 것이다. 이후 callback함수를 수정해준다. 
```python
    def callback_pose(self, msg):
        self.cmd_pose.pose_x = msg.x
        self.cmd_pose.pose_y = msg.y
        self.cmd_pose.linear_vel = msg.linear_velocity
        self.cmd_pose.angular_vel = msg.angular_velocity
    
    def callback_cmd(self, msg):
        self.cmd_pose.cmd_vel_linear = msg.linear.x
        self.cmd_pose.cmd_vel_angular = msg.angular.z
        
        print(self.cmd_pose)
```
이후 코드는 다음과 같이 완성된다.      
<img width="1178" height="982" alt="image" src="https://github.com/user-attachments/assets/f37f8b9e-5a33-444a-be2b-c772a3d35fa2" />         

colcon build 이후 turtlesim_node와 저번에 만든 my_publisher노드와 현재 만든 turtle_cmd_and_pose노드를 실행한다.        
<img width="1347" height="856" alt="image" src="https://github.com/user-attachments/assets/fdc49af7-2680-4427-b2e6-b03028c240af" />        

잘 작동하는 모습을 확인 할 수 있다. rqt_graph를 확인해 보면 다음과 같다.     
<img width="824" height="444" alt="image" src="https://github.com/user-attachments/assets/15dccdf0-6f07-41ee-8ede-15a8e82f7694" />       

## 2.6 하나의 노드에서 두 개 이상의 토픽 다루기 2
이제까지 토픽 두 개를 구독하는 것을 해보았다. 그 결과를 새로운 메시지 타입에 발행하는 것 해보자.        
publish를 일정한 주기로 하기 위해서 timer로 만든다. 1초에 한번씩 timer_callback함수를 실행하는 timer와 CmdAndPoseVel타입인 /cmd_and_pose에 퍼블리쉬하는 __init__에 다음을 추가해준다. 
```python
self.timer_period = 1.0
self.publisher = self.create_publisher(CmdAndPoseVel, '/cmd_and_pose', 10)
self.timer = self.create_timer(self.timer_period, self.timer_callback)
```
이후 timer_callback함수를 만들어준다. call_back_cmd에서는 print를 없앤다.     
```python
def timer_callback(self):
    self.publisher.publish(self.cmd_pose)
```
이후 저장해주고 colcon build후 실행해보면 다음과 같이 나타나는 것을 확인해 볼 수 있다. 
<img width="1579" height="931" alt="image" src="https://github.com/user-attachments/assets/4164485e-62c1-4963-badb-b9ea7cd29166" />          

이제 echo를 통해 메시지가 잘 발행되는 지 확인해보자. 
```
ros2 topic echo /cmd_and_pose
```
<img width="2042" height="969" alt="image" src="https://github.com/user-attachments/assets/09f52789-87cd-44b8-9b68-6ad6975981c8" />        

잘 발행되고 있는 모습을 확인해 볼 수 있다.    
## 2.7 서비스 메시지 정의 만들기
이번에는 서비스 정의를 만들어보자.     
이전에 만들어둔 my_first_package_msgs폴더에 srv 폴더를 만든다. 이후 srv폴더에 MuldiSpawn.srv파일을 만들어준다. 그리고 다음과 같이 만들어준다.   
```
int64 num
---
float64[] x
float64[] y
float64[] theta
```
여기서 대괄호는 배열이다. 우리는 turtlesim 노드의 서비스를 사용할 때 이 서비스의 definition을 알았어야 한다. 그때의 definition이 위의 내용과 같이      
```
request     
---    
response    
```
였다. 서비스 정의는 **request와 response를 구분**해야한다는 것을 기억하자. 이제 CMakeLists.txt를 수정한다. 지난번에 만든 "msg/CmdAndPoseVel.msg"밑에 다음의 줄을 추가한다.     
```txt
"srv/MultiSpawn.srv"   
```
package.xml도 수정해야하는데 저번에 msg 정의를 할때 만들어두었다. 이후 colcon build해준다.다시 환경을 부르고 interface show를 통해 확인해보자. 
```
ros2 interface show my_first_package_msgs/srv/MultiSpawn 
```
```
int64 num
---
float64[] x
float64[] y
float64[] theta
```
잘 나오는 것을 확인할 수 있다.    
## 2.8 서비스 서버 만들기
서비스 정의를 만들었으니 이제 이것을 사용할 서버를 만든다.     
my_first_package에서 my_service_sever.py를 만들자.     
```python
from my_first_package_msgs.srv import MultiSpawn
import rclpy as rp
from rclpy.node import Node

class MultiSpawning(Node):
    
    def __init__(self):
        super.__init__('multi_spawn')
        self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)
        
    def callback_service(self, request, response):
        print('Request : ', request)
        
        response.x = [1., 2., 3.]
        response.y = [10., 20.]
        response.theta = [100., 200., 300.]
        
        return response
def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()
    
if __name__ == '__main__':
    main()
```
MultiSpawn을 import해주고 class에서 create_service라는 명령을 사용햇다. 이게 서비스 서버를 만드는 명령이다. 여기서 어떤데이터타입을 사용할 것이고, 서비스 이름은 무엇이고,또한  서비스를 request한다면 실행해야하는 callback함수를 넣어준다. 우리가 만든 callback함수에서는 x,y,theta를 배열형태로 response해준다. 그 후 setup.py에 가서 한줄 추가해준다. 
```python
'my_service_server = my_first_package.my_service_server:main'
```
이제 빌드 후 서비스를 실행하고 service list를 확인해보면 다음과 같이 나오는 것을 확인해볼 수 있다.        
<img width="1353" height="664" alt="image" src="https://github.com/user-attachments/assets/1914a931-0c50-44a3-a506-6aaf15e47b8d" />         

이제 서비스를 킨 상태에서 다른 터미널에서 service call을 통해 num을 줘 보자.    
```
ros2 service call /multi_spawn my_first_package_msgs/srv/MultiSpawn "{num: 1}"
```
<img width="1361" height="431" alt="image" src="https://github.com/user-attachments/assets/a21365e7-99be-4776-a660-b4b3d103de54" />      
response를 받는 것을 확인해 볼 수 있다.     
이제 이 코드에 살을 붙여 나가보자!!
## 2.9 서비스 서버 만들기 응용편
우리가 만들고 있는 서비스 서버에 teleport_absolute에 대한 클라이언트를 만들어볼 것이다. 
```python
from my_first_package_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute

import rclpy as rp
from rclpy.node import Node

class MultiSpawning(Node):
    
    def __init__(self):
        super().__init__('multi_spawn')
        self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.req_teleport = TeleportAbsolute.Request()
        
    def callback_service(self, request, response):
        self.req_teleport.x = 1.
        self.teleport.call_async(self.req_teleport)
        
        return response
    
def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()
    
if __name__ == '__main__':
    main()
```
create_client를 만들어준다. 타입은 TeleportAbsolut이며 이름은 /turtle1/teleport_absolute이다. 그리고 req_teleport에 TeleportAbsolute.Request()를 잡아줬다. 그리고 callback_service에서 call_async를 해주면 이동하게 된다. 이제 빌드를 하고 turtlesim을 실행하고 my_service_server를 실행하고 다음과 같이 call해주면        
<img width="1360" height="855" alt="image" src="https://github.com/user-attachments/assets/9c6a1243-c28a-4fda-90a8-7485a0ff96a4" />     

서비스 서버 안에서 서비스 클라이언트를 만들수 있다는 것이 이번 코드에서 중요한 점이다. 
## 2.10 원하는 위치에 turtle 배치하는 알고리즘 작성하기
이제 거북이 원으로 배치하기 전에 알고리즘을 만들어야한다.     
8개의 거북이를 다음과 같이 배치하고 싶다면, 좌표계산을 어떻게 계산해야할지 알고리즘을 짜야한다.    
<img width="475" height="456" alt="image" src="https://github.com/user-attachments/assets/38f26095-5abf-4062-b106-63f70ddc86f7" />      

이때 jupyter note북을 이용해서 좌표를 구해보자. 
<img width="886" height="429" alt="image" src="https://github.com/user-attachments/assets/80b90f6e-bbc3-41e2-9ee8-2a579d1ddaf3" />       

<img width="853" height="279" alt="image" src="https://github.com/user-attachments/assets/4a10566b-bcad-4481-85a1-b6c59fb5008d" />       

<img width="853" height="279" alt="image" src="https://github.com/user-attachments/assets/6458974f-fbd4-4fa8-abad-da8052409584" />     

<img width="864" height="376" alt="image" src="https://github.com/user-attachments/assets/cdc28daa-53ea-40ed-b096-25db0031d62f" />    

<img width="894" height="491" alt="image" src="https://github.com/user-attachments/assets/37c3123d-46df-4be0-93b2-628c5073e684" />    

이렇게 좌표를 구할 수 있다. 

<img width="814" height="337" alt="image" src="https://github.com/user-attachments/assets/b29f5445-574b-42e2-ae2a-06dcdc706239" />

<img width="814" height="337" alt="image" src="https://github.com/user-attachments/assets/f06dcf5f-1580-4950-8c49-e66bc35c6c42" />

<img width="796" height="478" alt="image" src="https://github.com/user-attachments/assets/85f26bc3-7b73-4250-bf99-62b8ca941dd3" />    

## 2.12 다수의 서비스 클라이언트 구현하기
위에서 만든 알고리즘을 이용해 서비스를 만들어보자. 
import부분
```python
from my_first_package_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn

import time
import rclpy as rp
import numpy as np
from rclpy.node import Node
```
MultiSpawning 클래스의 init
```python
def __init__(self):
    super().__init__('multi_spawn')
    self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)
    self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
    self.spawn = self.create_client(Spawn, '/spawn')
    self.req_teleport = TeleportAbsolute.Request()
    self.req_spawn = Spawn.Request()
    self.center_x = 5.54
    self.center_y = 5.54
```
Spawn타입의 /spawn서비스를 사용하는 client를 만들어주었다. 그리고 center값을 잡아준다. 이후 위에 chapther에서 만든 알고리즘을 함수로 넣어준다.    
```python
def calc_position(self, n, r):
    gap_theta = 2*np.pi / n
    theta = [gap_theta * n for n in range(n)]
    x = [r*np.cos(th) for th in theta]
    y = [r*np.sin(th) for th in theta]
        
    return x, y, theta
```
이제 callback함수를 만든다. 
```python
    def callback_service(self, request, response):
        x, y, theta = self.calc_position(request.num, 3)
        
        for n in range(len(theta)):
            self.req_spawn.x = x[n] + self.center_x
            self.req_spawn.y = y[n] + self.center_y
            self.req_spawn.theta = theta[n]
            self.spawn.call_async(self.req_spawn)
            time.sleep(0.1)
            
        response.x = x
        response.y = y
        response.theta = theta
        
        return response
```
서비스 서버에 client가 달라붙었을때 request할 때 실행되는 함수이다. 마지막으로 main함수를 만든다.       
```python
def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()
    
if __name__ == '__main__':
    main()
```
이후 빌드하고 turtlesim을 실행하고 서비스 실행하고 서비스를 호출해보자. 
```
ros2 service call /multi_spawn my_first_package_msgs/srv/MultiSpawn "{num: 5}"
```
num: 5로 했기에 5마리가 소환되는 것을 확인해 볼 수 있다.    
<img width="1374" height="864" alt="image" src="https://github.com/user-attachments/assets/2ef32668-8972-48c4-a73d-d7e034bb2277" />    


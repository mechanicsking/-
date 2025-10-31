## 3.1 액션 메시지 정의 만들기
액션과 익숙해지기     
액션도 마찬가지로 액션의 정의를 만들어야한다.    
<img width="960" height="540" alt="image" src="https://github.com/user-attachments/assets/c7c0b054-6dbb-4c88-8f3e-8cad94c28947" />       

액션 client가 액션 server에게 send goal을 할때 액션의 정의를 보내야한다. 이때 request하는 데이터 타입이 같이 들어가야한다.      
my_first_package_msgs에 action 폴더를 만든다. 그리고 액션 폴더에서 DistTurtle.action이라는 새파일을 만들자      
액션은 세 영역으로 구분해야한다.     
<img width="193" height="131" alt="image" src="https://github.com/user-attachments/assets/225afc27-4108-4187-9417-1ca15526c17c" />       

```
# Request
float32 linear_x
float32 angular_z
float32 dist
---
# Result
float32 pos_x
float32 pos_y
float32 pos_theta
float32 result_dist
---
# Feedback
float32 remained_dist
```
이번 aciton의 컨셉은 turtlesim을 내가 원하는 거리만큼 이동시키는 것이다. request영역에는 선속도와 각속도와 이동해야하는 목표거리를 입력받는다. 선속도와 각속도는 유저에게 받은 후 cmd_vel 토픽으로 쏠거다. result로 받아올건 현재의 position과 자세아다. 이거는 pose토픽에서 받아올거다.       
이제 CMakeList에 만든 것을 추가해준다. 
```txt
"action/DistTurtle.action"
```
package.xml에도 다음의 줄을 한개 추가해준다. 
```xml
  <depend>action_msgs</depend>
```
이제 colcon build해주고 aciton정의를 interface show해본다. 
```
ros2 interface show my_first_package_msgs/action/DistTurtle
```
```
# Request
float32 linear_x
float32 angular_z
float32 dist
---
# Result
float32 pos_x
float32 pos_y
float32 pos_theta
float32 result_dist
---
# Feedback
float32 remained_dist
```
잘 나오는 것을 확인해 볼 수 있다. 
## 3.3 초간단 액션 서버 만들기   
먼저 dist_turtle_action_server.py라는 파일을 만들자. 
```python
import rclpy as rp
from rclpy.action import ActionServer
from rclpy.node import Node

from my_first_package_msgs.action import DistTurtle

class DistTurtleServer(Node):
    
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self._action_server = ActionServer(
            self, 
            DistTurtle,
            'dist_turtle',
            self.execute_callback)
        
    def execute_callback(self, goal_handle):
        goal_handle.succeed()
        result = DistTurtle.Result()
        return result
    
def main(args=None):
    rp.init(args=args)
    dist_turtle_action_server = DistTurtleServer()
    rp.spin(dist_turtle_action_server)
    
if __name__ == '__main__':
    main()
```
이렇게 action server의 틀을 잡아보았다. 이후 setup.py에 다음의 코드를 추가하자.  
```python
'dist_turtle_action_server = my_first_package.dist_turtle_action_server:main'
```
이후 빌드를 해주고 action server를 가동해준다. 이후 새로운 터미널을 열어 send_goal을 해준다.    
```
ros2 action send_goal /dist_turtle my_first_package_msgs/action/DistTurtle "{linear_x: 0, angular_z: 0, dist: 0}"
```
다음과 같은 결과가 출력된다. 
```
Waiting for an action server to become available...
Sending goal:
     linear_x: 0.0
angular_z: 0.0
dist: 0.0

Goal accepted with ID: 45589e26f6594b42b5468c7b383d5c3d

Result:
    pos_x: 0.0
pos_y: 0.0
pos_theta: 0.0
result_dist: 0.0

Goal finished with status: SUCCEEDED
```
## 3.5 액션에서 feedback 구현해보기
이번엔 feedback이라는 것을 받아보자.  
먼저 time을 import해주고 execute_callback함수르 다음과 같이 추가해준다.      
```python
    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback()
        for n in range(0, 10):
            feedback_msg.remainde_dist = float(n)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
            
        goal_handle.succeed()
        result = DistTurtle.Result()
        return result
```
callback함수에서 feedback_msg가 DistTurelt.Feedback을 잡도록 만들어주고 for문을 통해 float형으로 바꾸고 remainded_dist에다가 넣어주게 된다. 그걸 이제 publish_feedback시켜주면 된다. 이제 빌드하고 send_goal을 할때 --feedback옵션을 주면 된다.      
이제 빌드해주고, dist_turtle_action_server를 켜준 후, 다른 터미널에서 액션을 실행해준다. 
```
ros2 action send_goal --feedback /dist_turtle my_first_package_msgs/action/DistTurtle "{linear_x: 0, angular_z: 0, dist: 0}"
```
```
Waiting for an action server to become available...
Sending goal:
     linear_x: 0.0
angular_z: 0.0
dist: 0.0

Goal accepted with ID: 650619cd7835479fbd5c11d1e900d1c1

Feedback:
    remained_dist: 0.0

Feedback:
    remained_dist: 1.0

Feedback:
    remained_dist: 2.0

Feedback:
    remained_dist: 3.0

Feedback:
    remained_dist: 4.0

Feedback:
    remained_dist: 5.0

Feedback:
    remained_dist: 6.0

Feedback:
    remained_dist: 7.0

Feedback:
    remained_dist: 8.0

Feedback:
    remained_dist: 9.0

Result:
    pos_x: 0.0
pos_y: 0.0
pos_theta: 0.0
result_dist: 0.0

Goal finished with status: SUCCEEDED
```
잘 동작하는 것을 확인할 수 있다. 
## 3.6 멀티 쓰레드 사용해 보기
액션 콜백이 돌면 subscriber의 콜백이 돌지 않는 상황이 생긴다. 이때의 해결방안은?? --> 멀티쓰레드!      
```python
import rclpy as rp
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from my_first_package.my_publisher import TurtlesimPublisher
from my_first_package.my_subscriber import TurtlesimSubscriber

def main(args=None):
    rp.init()
    
    sub = TurtlesimSubscriber()
    pub = TurtlesimPublisher()
    
    executor = MultiThreadedExecutor()
    
    executor.add_node(sub)
    executor.add_node(pub)
    
    try:
        executor.spin()
        
    finally:
        executor.shutdown()
        #sub.destroy_node()
        pub.destroy_node()
        rp.shutdown()
        
if __name__ == '__main__':
    main()
```
먼저 rclp.executors에서 MultiThreadedExecutor을 가지고 온다. 그리고 저번에 만들어둔 TurtlesimPublisher와 TurtlesimSubscriber를 import해준다. 그리고 sub와 pub를 객체화 해주고, executor도 MultiThreadedExecutor로 객체화 해준다. 그리고 executor에 add_node를 통해 sub와 pub을 추가해준다. 그리고 executor를 spin해주면 된다. 코드작성이 완료되면, setup.py에 my_multi_thread = my_first_package.my_multi_thread:main' 를 추가해준다.       
colcon build를 하고 작동시켜주면 다음과 같이 publish와 subscribe가 함께 작동하는 것을 확인할 수 있다. 
<img width="1358" height="786" alt="image" src="https://github.com/user-attachments/assets/4a1ccd2c-cd9c-4537-af22-748e791ba39a" />

rqt_graph도 잘 나오고 있다.      
<img width="1145" height="645" alt="image" src="https://github.com/user-attachments/assets/f4a42633-236a-43fa-afe0-a7825a7b5f9f" />     

이렇게하면, callback이 너무 복잡하거나 실행시간이 오래걸려서 다른 callback을 방해할 때, 해결책이 될 수 있다. 
## 3.8 일정한 거리를 이동시키는 액션 서버 구현하기 1
이제 멀티 쓰레스까지 이용해서 지정한 거리만큼 거북이를 보내보자.      
먼저 import를 한다. 
```python
import rclpy as rp
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_first_package_msgs.action import DistTurtle
from my_first_package.my_subscriber import TurtlesimSubscriber

import math
import time
```
다음은 main함수이다. main함수에서 멀티쓰레드를 사용할 것이다. 
```python
def main(args=None):
    rp.init(args=args)
    
    executor = MultiThreadedExecutor()
    
    ac = DistTurtleServer()
    sub = TurtleSub_Action(ac_server = ac)
    
    executor.add_node(sub)
    executor.add_node(ac)
    
    try:
        executor.spin()
        
    finally:
        executor.shutdown()
        sub.destroy_node()
        ac.destroy_node()
        rp.shutdown()
```
main함수에선 executor를 MultiThreadedExecutor객체로 만들준다. 그리고 DistTurtleServer객체 ac와 이따 만들 TurtleSub_Action객체 sub을 executor에 add_node를 해준후 try를 통해 executor.spin을 해준다.  이제 turtlesub_action을 살펴보자. 
```python
class TurtleSub_Action(TurtlesimSubscriber):
    def __init__(self, ac_server):
        super().__init__()
        self.ac_server = ac_server
        
    def callback(self, msg):
        self.ac_server.current_pose = msg
```
TurtleSub_Action은 TurtelsimSubscriber를 상속받는다. 그리고 __init__()을 상속받고 self.ac_server를 추가해준다. 이후 callback함수에서는 ac_server받아온거에 current_pose라는 속성을 만들어두고 여기에 msg를 집어넣을 것이다. 함수의 오버리이딩      
이후 DistTurtleServer의 init을 확인해보자. 
```python
class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0
        self.is_first_time = True
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._action_server = ActionServer(
            self, 
            DistTurtle,
            'dist_turtle',
            self.execute_callback)
```
## 3.8 일정한 거리를 이동시키는 액션 서버 구현하기 2
이제 DistTurtleServer의 calc_diff_pose함수이다. 
```python
    def calc_diff_pose(self):
        if self.is_first_time:
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            self.is_first_time = False
        
        diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x)**2 +
                              (self.current_pose.y - self.previous_pose.y)**2)
        
        self.previous_pose = self.current_pose
        
        return diff_dist
```
turtle이 지나갈때 과거의 pose와 현재의 pose를 통해 이동한 거리를 diff_dist에 저장하여 retrun해준다. 이제 callback함수를 확인한다.
```python
    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback()
        
        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z
        
        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remained_dist = goal_handle.request.dist - self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.publisher.publish(msg)
            time.sleep(0.01)
            
            if feedback_msg.remainde_dist < 0.2:
                break
            
        goal_handle.succeed()
        result = DistTurtle.Result()
        
        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = self.total_dist
        
        self.total_dist = 0
        self.is_first_time = True
        
        return result
```
빌드를 하고 turtlesim_node를 실행하고 액션서버를 실행하고 다른 터미널에서 액션 서버에 다음의 명령어를보내면 다음과 같은 결과가 나타난다. 
```
ros2 action send_goal --feedback /dist_turtle my_first_package_msgs/action/DistTurtle "{linear_x: 0.8, angular_z: 0.4, dist: 2.}"
```
<img width="1347" height="858" alt="image" src="https://github.com/user-attachments/assets/46a3830c-418a-478b-b9f6-66045288b7e5" />       

rqt_graph를 확인해보면 다음과 같다.     
<img width="1148" height="644" alt="image" src="https://github.com/user-attachments/assets/91c53a1e-ae35-449e-8e31-c388e6a0a3d0" />

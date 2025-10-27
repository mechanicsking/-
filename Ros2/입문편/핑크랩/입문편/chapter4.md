## 4.1 Python으로 topic 구독 해보기  
ros2 humble 환경을 불러온 후 다시 jupyter notebook과 turtlesim_node 실행      
<img width="1740" height="662" alt="image" src="https://github.com/user-attachments/assets/dd0720e9-f2b8-435e-9584-a4913d9f8257" />         

pose토픽을 한번 구독해보자, 구독하기 위해선 토픽의 이름과 타입을 알아야한다.      
따라서 다음과 같이 먼저 작성한다. 
```python
import rclpy as rp
from turtlesim.msg import Pose
```
rclpy는 ros common library for python이다. 이를 rp라 하며 불러온다. 그리고 turtlesim.msg의 Pose도 import한다. python에서는 데이터 타입을 import해서 가지고 온다!!      
그리고 create_node를 한다.  
```python
rp.init()
test_node = rp.create_node('sub_test')
```
rp를 먼저 init시키고 rp가 제공하는 create_node를 사용하여 sub_test라는 노드를 만든다. 그리고 그걸 test_node라는 이름으로 받았다. 이제 여기서 node list를 확인해본다.         
<img width="1358" height="825" alt="image" src="https://github.com/user-attachments/assets/2b5ce5ba-42d9-41a9-bf53-cabd0e6b6d3b" />       

sub_test노드가 잘 실행되고 있는 것을 확인해볼 수 있다.      
이제 callback 함수를 작성해 줄 것이다. subscription은 토픽이 들어올 때마다 실행하는 함수를 하나 지정해야한다. 우리는 그것을 callback이라고 말 할 것이다. (topic을 받을 때마다 어떤 일을 수행하게 하는 함수)
```python
def callback(data):
    print("--->")
    print("/turtle1/pose : ", data)
    print("X : ", data.x)
    print("Y : ", data.y)
    print("Theta : ", data.theta)
```
그리고 create_subscription해야한다. 
```python
test_node.create_subscription(Pose, '/turtle1/pose', callback, 10)
```
Pose라는 데이터 타입을 쓰는 /turtle1/pose라는 토픽을 구독하라고 아까 만든 test_node에게 시킨다. 그리고 그 topic이 들어올때마다 callbaack함수를 실행하라고 했다.      
이때 rqt_graph를 실행하자.     
<img width="832" height="453" alt="image" src="https://github.com/user-attachments/assets/4e134123-7439-43f4-9652-87f0f669aec1" />       

```python
rp.spin_once(test_node)
```
<img width="1112" height="135" alt="image" src="https://github.com/user-attachments/assets/6f8272c4-e1e2-44e3-828f-6bdc6fb0de07" />      

topic이 한번 들어오는 것을 볼 수 있다. 

```python
rp.spin(test_node)
```
spin하면 토픽이 계속 들어온다.       
<img width="1134" height="357" alt="image" src="https://github.com/user-attachments/assets/b3c71825-b419-40e2-b2b8-0dd5f723b14f" />        

## 4.2 Topic을 받는 횟수 제한하기
topic의 횟수를 제한하는 가장 쉬운방법은 count를 제한한다. callback함수부분을 다음과 같이 바꾸어준다. 
```python
cnt = 0
def callback(data):
    global cnt
    cnt += 1
    print(">", cnt, " -> X : ", data.x, ", Y : ", data.y)

    if cnt > 3:
        raise Exception("Subscription Stop!")
```
이후 다음의 코드 실행
```python
rp.spin(test_node)
```
<img width="1134" height="357" alt="image" src="https://github.com/user-attachments/assets/2929dbed-f2fd-4e71-b44e-f06b832df90f" />     
잘 작동하는 것을 확인해 볼 수 있다.    

## 4.3 ROS2 topic 발행해보기
이번에는 publisher test로 만들어준다. 우리는 cmd_vel 토픽을 발행해 볼것이다. 
```python
import rclpy as rp
from geometry_msgs.msg import Twist

rp.init()
test_node = rp.create_node('pub_test')
```
다음으로 Twist의 데이터 타입을 가져와 msg라는 것으로 객체화한다. 
```python
msg = Twist()
print(msg)
```
이렇게 하면 다음과 같이 print되는 것을 볼 수 있다. 
```
geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```
다음과 같이 바꿀 수 있다. 
```python
msg.linear.x = 2.0
print(msg)
```
```
geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```
실행은 다음과 같이한다.     
```python
pub = test_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
pub.publish(msg)
```
이렇게 만들어주면 한번 간다.    
<img width="503" height="533" alt="image" src="https://github.com/user-attachments/assets/191cdd1f-ace9-4835-98e6-5adc1b6b5ec6" />       

일정한 시간동안 보내고 싶다면??--> 타이머를 이용한 콜백을 한개 만든다. 
```python
cnt = 0

def timer_callback():
    global cnt

    cnt += 1

    print(cnt)
    pub.publish(msg)

    if cnt > 3:
        raise Exception("Publisher Stop")
```
타이머를 선언해서 사용한다.     
```python
timer_period = 0.1
timer = test_node.create_timer(timer_period, timer_callback)
rp.spin(test_node)
```
그리고 노드를 종료할 땐 다음의 코드를 실행한다. 
```python
test_node.destroy_node()
```

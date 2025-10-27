## 5.1 ROS2 service client 다루기 1
python으로 ros 클라이언트 사용하기      
우리는 turtle1/teleport_absolute서비스를 사용해보자.      
client_test 노드를 만들어보자.   
```python
import rclpy as rp
from turtlesim.srv import TeleportAbsolute

rp.init()
test_node = rp.create_node('client_test')
```
create_node란??뭘 사용할 수 있는가??--> ros2 create_node항목 공식 문서를 보자!( https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html )
create_node는 rclpy.node.Node를 return한다. 따라서 위의 코드에서 test_node는 rp.create_node('client_test')여기에서 return받은 것이 들어갔기에 얘의 클래스는 rclpy.node이다. 따라서 rclpy의 node를 찾아보면 다음과 같이 나와았다.    
<img width="704" height="780" alt="image" src="https://github.com/user-attachments/assets/320c0eaf-3a4a-4f89-a555-daa05bbbf298" />      

밑을 내려보면 create_client문서를 확인해 볼 수 있다.     
<img width="671" height="298" alt="image" src="https://github.com/user-attachments/assets/5e867fd5-2074-4b9a-ae94-85b0aa4356cc" />

따라서 위에서 우리는 create_node를 이용해서 노드를 만들어서 Node를 return받았기 때문에 여기에 해당하는 것들을 사용할 수 있다.      
```python
test_node.create_client
```
이제 create_client를 사용한다. 
```python
service_name = '/turtle1/teleport_absolute'
cli = test_node.create_client(TeleportAbsolute, service_name)
```
TeleportAbsolute타입을 사용하는 /turtle1/teleport_absolute서비스 를 사용하겠다. 라는 의미이다. 
서비스 정의 다시 확인    
```
request
---
response
```
이제 이렇게 서비스 정의를 사용할 준비를 해준다. 
```python
req = TeleportAbsolute.Request()
req
```
```
turtlesim.srv.TeleportAbsolute_Request(x=0.0, y=0.0, theta=0.0)
```
값을 다음과 같이 변경한다. 
```python
req.x = 1.
req.y = 1.
req.theta = 3.14

req
```
```
turtlesim.srv.TeleportAbsolute_Request(x=1.0, y=1.0, theta=3.14)
```
이제 sevice를 call한다. 
```python
req.x = 3.

cli.call_async(req)
rp.spin_once(test_node)
```
<img width="1651" height="557" alt="image" src="https://github.com/user-attachments/assets/68e21905-3c23-4669-bf6c-8df6972062b9" />          

파이썬에서 서비스를 request하는 모습이다.     
wait_for_service : 서비스가 만들어질때까지 기다려줌  
```python
req.y = float(9)

while not cli.wait_for_service(timeout_sec=1.0):      //서비스가 준비될때까지 기다리라
  print("Waiting for service")

cli.call_async(req)
rp.spin_once(test_node)
```
서비스가 실행되는 결과 받아오기 
```python
req.x = float(9)

future = cli.call_async(req)

while not future.done():
  rp.spin once(test_node)
  print(future.done(), future.result())
```
call async를 한 것을 future라고 받고 future의 속성으로는 서비스서버가 일을 다하면 done을 반환해준다. 

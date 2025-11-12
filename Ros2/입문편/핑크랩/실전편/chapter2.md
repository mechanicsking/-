## 2.1 간단한 TF를 Broadcaster 해보기
좌표계를 발행해보자-TF
복잡한 좌표계를 해석해주는 도구가 있다. 
먼저 home에서 코드 작성을 위한 워크 스페이스를 만들자. 
```
mkdir -p my_tf_tutorials/src
```
src를 열어준다. 
```
cd my_tf_tutorials/src/
```
그리고 pkg를 만들어준다. 
```
ros2 pkg create --build-type ament_python my_tf
```
그리고 my_tf에 my_ft_1.py라는 파일을 만들어 다음 코드를 추가해준다.   
```python
# sudo apt install ros-jazzy-tf-transformations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
import tf2_ros

class MyTfBroadcaster(Node):
    def __init__(self):
        super().__init__('my_tf_broadcaster')
        
        # TransformBroadcaster 생성
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # 타이머 설정(0.1초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 각도 t를 0으로 초기화
        self.t = 0.0

    def timer_callback(self):
        # t를 조금씩 증가
        self.t += 0.05

        # 반지름 2m인 원 위를 돌도록 x, y 계산
        x = 2.0 * math.cos(self.t)
        y = 2.0 * math.sin(self.t)
        z = 0.0

        # TransformStamped 메시지 생성
        t = TransformStamped()
        
        # 현재 시간 정보
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 부모 프레임과 자식 프레임 설정
        t.header.frame_id = 'world'
        t.child_frame_id = 'moving_frame'
        
        # 위치 설정
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # TF 브로드캐스트
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MyTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
setup.py의 console_scripts에 'my_tf_1 = my_tf.my_tf_1:main',를 등록해준다. 이후 빌드해주고 소스 하고 실행한후 rviz2를 작동하면 다음과 같이 잘 작동하는 것을 확인해볼 수 있다.       
<img width="863" height="844" alt="image" src="https://github.com/user-attachments/assets/a88263ab-9737-4981-acf8-aaec6d7c457d" />       

rqt에서 TF tree를 확인해보면 다음과 같다.      
<img width="973" height="708" alt="image" src="https://github.com/user-attachments/assets/8f877522-fdf7-438b-8748-e2b05d9aba85" />       

 ## 2.2 TF에서 자세 설정해보기 (Feat.Quaternion)
 좌표계를 표현할 때 자세(pose)도 신경써보자      
 자세를 표현해보자.    
 2D에서 (x, y)를 위치라 한다면, theta는 자세이다. 공간에선?? 위치:(x, y, z), 자세:(yaw, pitch, roll)       
 오일러 방법은 회전을 설명할 때 회전 순서에 따라 결과가 달라진다.     
 <img width="849" height="511" alt="image" src="https://github.com/user-attachments/assets/ef792169-c98a-4689-a3ee-102edb1f663c" />        

회전을 순차적으로 하는 오일러 방법은 gymbal lock 현상이 생길 수 있다.     
<img width="967" height="335" alt="image" src="https://github.com/user-attachments/assets/fd9c9e32-74a9-4ce5-be08-ee23618b9f42" />      

<img width="967" height="335" alt="image" src="https://github.com/user-attachments/assets/2ae3aa98-cb22-4ffc-8542-9044b8c7e792" />     

<img width="664" height="585" alt="image" src="https://github.com/user-attachments/assets/8c747786-8973-4e9c-898b-961b9bfdc167" />         

이제 우리도 자세를 변화시켜 보자.    
위에서 만들었던 moving_frame을 확인해보면 원을 그리며 움직이지만, 자세가 바뀌지는 않는다. 이번에는 moving_frame이 어느 위치에 있는 world를 바라보게 해보도록 자세를 만들어볼 것이다. 
저번 코드에서 MyTfBroadcaster에 callback에 world를 보도록 자세를 만들고 quaternion으로 변환한다.    
먼저 quaternion을 사용할 수 있게 import해준다. 
```python
from tf_transformations import quaternion_from_euler
```
```python
        yaw = math.atan2(-y,-x)
        
        q = quaterniaon_from_euler(0.0, 0.0, yaw)
```
원의 중심 (0, 0)을 바라보도록 yaw 계산한다. (x, y) -> (0, 0)의 방향 벡터는 (-x, -y), 따라서 math.atan2(-y,-x)를 이용해 yaw값을 구해주었다. 그리고 오일러각 -> 쿼터니안 변환을 해주고자 q를 만들어주었다. 그 다음 위치에 더해 자세도 발행하도록 하였다.     
```python
        # 자세(쿼터니안) 설정
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
```
이후 빌드후 소스한 후 실행해보면 다음과 같이 x축이 world를 바라보며 회전하는 것을 확인할 수 있다.      
<img width="912" height="889" alt="image" src="https://github.com/user-attachments/assets/935747ce-9a0b-4b97-9b7a-7c0eb6a18d5b" />        

## 2.3 ROS TF에서 child frame 추가해보기 
child frame 하나 추가해보자.     
지난번에 만든 moving frame에 child frame을 붙여본다. 
```python
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros

class ChildTfBroadcaster(Node):
    def __init__(self):
        super().__init__('child_tf_broadcaster')
        
        # TransformBroadcaster 생성
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # 타이머 설정(0.1초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 각도 t를 0으로 초기화
        self.t = 0.0

    def timer_callback(self):
        # t를 조금씩 증가
        self.t += 0.05
        
        # 반지름 1m, 2배 빠른 각도(= 2 * t)
        angle = 2.0 * self.t
        
        # 부모 프레임인 moving_frame을 기준으로,
        # 반지름 1m 원을 그리면서 회전
        x = 1.0 * math.cos(angle)
        y = 1.0 * math.sin(angle)
        z = 0.0
        
        # (x,y)가 (0,0)에 대해 바라보는 방향(yaw) 계산
        # (원점 -> (x,y))의 반대방향: (x,y) -> (0,0) 는 (-x, -y)
        yaw = math.atan2(-y, -x)
        
        # 오일러 각 -> 쿼터니언 변환 (roll=0, pitch=0, yaw=계산값)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        
        # TransformStamped 메시지 생성
        t = TransformStamped()
        
        # 현재 시간 정보
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 부모 프레임과 자식 프레임 설정
        t.header.frame_id = 'moving_frame'
        t.child_frame_id = 'child_frame'
        
        # 위치 설정
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # 자세(쿼터니언) 설정
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # TF 브로드캐스트
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ChildTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
__init__함수에선 child_tf_broadcaster라는 node를 만들어주었고 TransformBroadcaster를 불러주고, 타이머를 설정해주었다. callback함수에선 그냥 방향을 지정하고 위치를 지정하였다. 그리고 frame_id와 child_frame_id를 잘 지정하면 좌표를 고민할 필요가 없어진다. 중요한것은 **parent_frame**과 **child_frame**을 잘 지정하면 된다는 것이다.  이후 setup.py에 'child_frame = my_tf.child_frame:main'를 추가 해주고, 빌드하고 소스해서, 원래의 moving_frame을 실행하고, child_frame을 실행하고, rviz를 실행해주면 다음과 같이 나오게 된다.     
<img width="1117" height="842" alt="image" src="https://github.com/user-attachments/assets/dea292cb-3779-4b25-bc46-91ee95ef844e" />      

tf tree를 확인해보면 다음과 같이 나오는 것을 확인해볼 수 있다.     
<img width="975" height="708" alt="image" src="https://github.com/user-attachments/assets/0fde165f-3885-4627-a9c9-3cbca18db1e7" />    

하지만 frame마다 코드를 만들 순 없으니 하나로 합쳐야한다.     
다음의 combined_frame.py 파일을 만든다.    
```python
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros


class CombinedTfBroadcaster(Node):
    def __init__(self):
        super().__init__('combined_tf_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.t = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def send_tf(self, parent, child, x, y, z, yaw):
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = parent
        t_msg.child_frame_id = child
        t_msg.transform.translation.x = x
        t_msg.transform.translation.y = y
        t_msg.transform.translation.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t_msg.transform.rotation.x = q[0]
        t_msg.transform.rotation.y = q[1]
        t_msg.transform.rotation.z = q[2]
        t_msg.transform.rotation.w = q[3]
        self.br.sendTransform(t_msg)

    def timer_callback(self):
        self.t += 0.05

        # world -> moving_frame: 반지름 2m 원, (0,0)을 바라보도록
        x1 = 2.0 * math.cos(self.t)
        y1 = 2.0 * math.sin(self.t)
        yaw1 = math.atan2(-y1, -x1)
        self.send_tf('world', 'moving_frame', x1, y1, 0.0, yaw1)

        # moving_frame -> child_frame: 반지름 1m 원, 2배 빠른 각도
        angle = 2.0 * self.t
        x2 = 1.0 * math.cos(angle)
        y2 = 1.0 * math.sin(angle)
        yaw2 = math.atan2(-y2, -x2)
        self.send_tf('moving_frame', 'child_frame', x2, y2, 0.0, yaw2)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
이번에는 send_tf라는 함수를 만들어준다. parant와 child, x, y, z, yaw값을 받는 함수이다. 이 함수에서 sendTransform하면 그만이다. 이제 timer_callback에서  send_tf를 통해 child_frame과 moving_frame을 넣어주면 된다. 그 후 setup.py에 'combined_frame = my_tf.combined_frame:main',를 추가해준다. 그렇게 한다면 똑같은 결과가 나오게 된다.    
## 2.4 ROS2 TF에서 frame 정보 구독해보기
TF frame을 구독하고 좌표계 사이의 정보를 활용하는 법       
오늘은 저번에 만들어둔 frame을 구독해서 frame과의 거리를 계산해볼 것이다.        
distance_world_child_publisher.py 파일을 만들어준다.      
```python
import math
import rclpy
from rclpy.node import Node

import tf2_ros
from std_msgs.msg import Float32

class DistanceWorldChildPublisher(Node):
    def __init__(self):
        super().__init__('distance_world_child_publisher')
        
        # TF를 조회하기 위한 Buffer와 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 거리 정보를 발행할 Publisher
        self.dist_pub = self.create_publisher(Float32, 'distance_world_child', 10)
        
        # 주기적으로 TF를 조회해 거리 계산 (0.1초마다)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # 'world'에서 'child_frame'으로의 변환 조회
            # lookup_transform(부모, 자식, 시간)
            transform = self.tf_buffer.lookup_transform(
                'world',       # 부모 프레임(출발)
                'child_frame', # 자식 프레임(도착)
                rclpy.time.Time()
            )

            # 변환으로부터 x, y, z 추출
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # 3차원 유클리드 거리 계산
            distance = math.sqrt(x**2 + y**2 + z**2)

            # 메시지 생성 및 발행
            msg = Float32()
            msg.data = distance
            self.dist_pub.publish(msg)

        except Exception as e:
            # 아직 TF가 준비되지 않았거나, lookup 실패할 경우 예외 발생
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceWorldChildPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

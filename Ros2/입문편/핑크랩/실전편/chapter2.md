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

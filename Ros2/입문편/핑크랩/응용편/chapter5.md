## 5.1 LOG 모니터링 하기
디버그와 관찰을 위한 도구들, 그리고 roslaunch       
왜 로그를 사용해야하는가??     
정상적인 정보, 경고, 에러 등등을 수준별로 관리하는 로그(log)에 기록을 남기면 추후 디버그에 아주 유리하다.       

다음으로 turtlesims노드를 실행하고 rqt를 실행하자.     
rqt화면에서 plugins-->Logging-->Console 선택      
꼭 rqt_plugin만을 통해 log를 관찰해야하는 건 아님!

이제 또다른 터미널에서 다음의 직진하는 topic 명령을 준다. 
```
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2., y: 0., z: 0.}, angular: {x: 0., y: 0., z: 0.}}"
```
벽에 부딪혔다고 경고가뜬다. 이것이 rqt log console에 기록이 남는다.      
<img width="1871" height="933" alt="image" src="https://github.com/user-attachments/assets/15709922-95bd-4a66-b500-b6788c168328" />       

## 5.3 LOG 기능 코드로 구현하기 
dist_turtle_action_server에 log메시지 출력 직접 만들기    
```python
        self.get_logger().info('Dist turtle action server is started.')
```
이 줄을 DistTurtleServer 클래스의 __init__함수에 넣어준다. 이제 빌드하고 다시 실행해보면...       
<img width="686" height="108" alt="image" src="https://github.com/user-attachments/assets/cbc8a4ac-1a91-44ff-824e-6ac43d8d85ab" />       

INFO가 잘 작동한다. 
또 메시지를 한번 추가해보자. parameter_callback함수 전에 다음 문장을 추가해준다. 
```python
        output_msg = "quantile_time is " + str(self.quantile_time) + ", "
        output_msg = output_msg + "and almost_goal_time is " + str(self.almosts_time) + ". "
        self.get_logger().info(output_msg)
```
또한 parameter_callback함수 에서 print문을 다음의 문장으로 바꾸어준다.        
```python
        output_msg = "quantile_time is " + str(self.quantile_time) + ", "
        output_msg = output_msg + "and almost_goal_time is " + str(self.almosts_time) + ". "
        self.get_logger().info(output_msg)
```
또한 execute_callback함수에서 time.sleep(0.01)전에 다음의 코드를 추가한다. 
```python
            tmp = feedback_msg.remained_dist - goal_handle.request.dist * self.quantile_time
            tmp = abs(tmp)
            
            if tmp < 0.02:
                output_msg = 'The turtle passes the ' + str(self.quantile_time) + ' point. '
                output_msg = output_msg + ' : ' + str(tmp)
                self.get_logger().info(output_msg)
```
이제 빌드하고 rqt를 실행하고, turtlsim_node를 실행하고 dist_turtle_action_server를 실행한다. 마지막으로 다음을 실행해주면     
```
ros2 action send_goal dist_turtle my_first_package_msgs/action/DistTurtle '{linear_x: 2., angular_z: 2., dist: 2,}'
```
다음과 같이 rqt에 나오게 된다.        
<img width="1650" height="1131" alt="image" src="https://github.com/user-attachments/assets/0fb4aa10-9df2-406b-9740-9b11f6f1b1e0" />       

이제 앞으로 로그를 이용해서 복잡한 시스템의 디버그를 사용할 때가 있을 것이다.    

## 5.5 RQT로 topic 다루기
이번에는 rqt를 가지고 topic이나 service를 관리, 발행을 해볼 것이다.    
먼저 다음과 같이 실행한다.    

<img width="2313" height="848" alt="image" src="https://github.com/user-attachments/assets/816250c7-810a-4d56-81a2-4952f84d56f8" />    

```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
그리고 나서 rqt에서 plugins-->visualization-->plot 을 선택한다.     
그 후 토픽부분에 turtle1/pose/를 작성해준다.       
그러면 다음과 같은 화면이 뜬다.     
<img width="941" height="671" alt="image" src="https://github.com/user-attachments/assets/7336ef0c-af31-4f57-bb70-8662b5edac8d" />      

이후 옆에 -를 통해 원하는 토픽은 선택하거나 제거할 수 있다. 다음과 같이 두개만 남긴후    
<img width="971" height="714" alt="image" src="https://github.com/user-attachments/assets/e0a2df7f-3e47-4b92-9936-35056da67073" />      

그 후 메뉴쪽에 세이브 옆에 주식창과 같은 모양을 클릭한 후
<img width="971" height="714" alt="image" src="https://github.com/user-attachments/assets/a2b365d7-b3f9-4495-916d-4497a575cf88" />       

이렇게 바꿔주면      
<img width="971" height="714" alt="image" src="https://github.com/user-attachments/assets/d5404d22-f567-47f1-81aa-7d73d3c748f1" />        

이렇게 토픽을 관찰할 수 있다.     

토픽 모니터링 기능도 있다.     
<img width="1293" height="636" alt="image" src="https://github.com/user-attachments/assets/db2a7b9d-3e26-45f7-ae56-9490904acec9" />      

이 gui로 보는 토픽 모니터링이 업데이트가 느리다. 그렇기에 빠른 속도로 변화하는 토픽에는 활용하기 어렵다. 사진 이미지는 rqt그래프로 보면 된다. 센서데이터가 point cloud방식이면 rviz에서 보면 봐야한다.      
topic publisher도 가능하다.    

<img width="983" height="717" alt="image" src="https://github.com/user-attachments/assets/94108c69-1e07-4b81-bdd1-a3dca525de90" />       

여기서 값수정을 하고 체크를 하면 발행이 된다. 

<img width="1048" height="717" alt="image" src="https://github.com/user-attachments/assets/d9974fac-5159-488b-9603-baf14c956926" />        

## 5.7 rosbag 사용법 
rosbag는 토픽을 저장하고 발행해준다.     
먼저 turtlesim_node를 실행해주고 topic을 쏜다.     
이후 다음을 이용해 record를 해주고 ctrl+c를 통해 멈춰준다.     
```
ros2 bag record -o turtle_test -a
```
이후 topic발행을 멈추고 ros bag을 play해준다. 
```
ros2 bag play turtle_test
```
그러면 turtlesim이 움직인다. 이것을 보고 싶다면 rqt에 rosbag을 이용하는 방법도 있다.        
<img width="977" height="719" alt="image" src="https://github.com/user-attachments/assets/e643ced6-4807-4331-bed6-0cebb681303a" />    

cmd_vel topic이 발행되고 있음을 알 수 있다.    

## 5.9 dynamic reconfigure에서 슬라이드바 만들기     
rqt로 파라미터 수정    
먼저 turtlesim_node를 실행하고 rqt를 실행      
rqt-->plugins-->configuration-->dynamic reconfigure 선택       
<img width="977" height="719" alt="image" src="https://github.com/user-attachments/assets/ccdcfe56-0d93-4e41-b77b-78a2fa406546" />        

이 슬라이드바로 색상을 바꿔볼 수 있다.     
<img width="1476" height="717" alt="image" src="https://github.com/user-attachments/assets/08228bc1-b849-4ec7-a224-92b63e0f4de7" />      

이 슬라이드를 추가하기 위해서는??? 다시 dist_turtle_action_server.py를 꾸며보자.    
먼저 필요한 모듈을 불러준다. 
```python
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
```
그리고 다음의 코드를 추가해준다.    
```python
        param_desc_quantile = ParameterDescriptor(
            description = 'quatile_time description',
            floating_point_range = [FloatingPointRange(
                from_value=0.0,
                to_value=1.0,
                step=0.01
        )]
        )
        
        
        self.declare_parameter('quantile_time', 0.75, param_desc_quantile)
```

이렇게 하고 저장하고 빌드해주면 다음과 같이 나오는 것을 확인할 수 있다.     
<img width="1702" height="860" alt="image" src="https://github.com/user-attachments/assets/1cc061c2-17bd-49b9-a3b2-a542851fbde9" />       

<img width="1702" height="860" alt="image" src="https://github.com/user-attachments/assets/ac2f9a34-0d1d-45ad-b3b6-aef8b3e2d1b0" />

잘 움직인다.    

## 5.10 ros2 launch
먼저 my_first_package에 launch라는 폴더를 하나 만든다.     
그리고 turtlesim_and_teleop.launch.py라는 이름의 파일을 하나 만들어 다음과 같이 작성한다.    
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= "turtlesim", package='turtlesim',
                executable='turtlesim_node', output='screen'),
            Node(
                namespace= "pub_cmd_vel", package='my_first_package',
                executable='my_publisher', output='screen'),
        ]
    )
```
generate_launch_description함수는 LaunchDescription이라는 것을 return한다. 이 안에는 node를 구성해볼 수 있다. turtlesim패키지의 turtlesim_node를 turtlesim이라는 이름으로 실행하라는 뜻이다. 또한 다음 노드는 my_first_package패키지에 들어있는 my_publisher라는 노드를 pub_cmd_vel이라는 이름으로 실행하라는 뜻이다. 그다음 setup.py에 다음 import 두줄을 먼저 추가하고   
```python
import os
import glob
```
이후 data_files에 launch 파일이 있다라는 것을 알려준다. 
```python
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py')))
```
그리고 my_publisher.py에서 publisher부분에서 /turtle1/cmd_vel를 '/turtlesim/turtle1/cmd_vel'로 바꿔준다.   이제 빌드를 하고 launch파일을 실행해준다.    
```
ros2 launch my_first_package turtlesim_and_teleop.launch.py 
```
<img width="1878" height="848" alt="image" src="https://github.com/user-attachments/assets/b87d2a4a-5dca-4da5-854c-7ac8b8d21c50" />      

두개의 노드가 잘 작동하는 것을 확인해 볼 수 있다.    
<img width="968" height="725" alt="image" src="https://github.com/user-attachments/assets/2c83a08f-7eaa-416d-94a3-3eb5797b78f1" />       

잘작동하는 것을 확인해 볼 수 있다.      
또, launch에서는 파라미터도 바꿀 수 있다.dist_turtle_action.launch라는 이름으로 새로운 launch파일을 만들어 새로운 스타일로 launch파일을 만든다.    
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    my_launch = LaunchDescription()
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
        parameters=[
            {"background_r": 255},
            {"background_g": 192},
            {"background_b": 203},
        ]
    )
    
    dist_turtle_action_node = Node(
        package='my_first_package',
        executable='dist_turtle_action_server', 
        output='screen'
    )
    
    my_launch.add_action(turtlesim_node)
    my_launch.add_action(dist_turtle_action_node)
    
    return my_launch
```
LaunchDescription을 선언하고 turtlesim_node는 Node함수를 이용해서 정의하고 dist_turtle_action_node도 정의해준다. 그 후 my_launch에 add_action이라는 이름으로 방금 잡아둔 두개의 변수르 추가해준다. 그리고 return해준다.  그리고 setup.py를 수정해서 원래는 py로 한정한 launch파일을 모든 파일이 가능하게 해준다.    
```python
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*')))
```
이후 빌드해주고 런치를 실행해주면
```
ros2 launch my_first_package dist_turtle_action.launch.py 
```
<img width="1196" height="687" alt="image" src="https://github.com/user-attachments/assets/fe4bc57e-649a-41f0-a6e4-4195a676d862" />      



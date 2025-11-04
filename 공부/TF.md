## TF(Transform system)
ros에서의 transform system을 TF2라고 한다. 사용자가 시간 경과에 따른 여러 좌표 프레임을 추적할 수 있도록 하는 Transform 라이브러리다. 
### TF가 필요한 이유??
로봇은 여러 부품으로 이루어져 있고, 각 부품은 **"자기 중심적"**입니다.    

Lidar 센서: "벽이 내 2미터 앞에 있어." (lidar_link 좌표계)    
로봇 팔: "공이 내 0.5미터 아래에 있어." (gripper_link 좌표계)     
로봇 본체(두뇌): "그래서 그 '벽'이랑 '공'이 나 (base_link) 기준으로 대체 어디 있는데?"     

로봇의 두뇌(base_link)는 이 모든 제각각인 좌표 정보를 "자신"의 기준으로 통합해야만 "벽을 피해 공을 잡는" 작업을 수행할 수 있습니다.  
### TF가 하는 일
TF는 이 모든 제각각인 좌표계(Frame)들의 관계를 하나의 거대한 트리(Tree) 구조로 묶어 관리합니다.     

발행 (Broadcast):     
robot_state_publisher 같은 노드가 로봇의 설계도(URDF/Xacro)를 읽습니다.     
"Lidar는 base_link로부터 (x=0.5, y=0)에 붙어있다"     
"팔꿈치는 어깨로부터 (z=0.3)에 붙어있다"     
...이런 "연결 관계(Transform)"를 1초에 수십 번씩 ROS 시스템 전체에 **방송(publish)**합니다.      

수신 및 계산 (Lookup):     
Nav2나 RViz 같은 노드는 이 TF 방송을 듣고 있습니다. 
Nav2가 "Lidar가 (2,0)에 벽을 봤다는데, 그거 base_link 기준으로는 어디야?"라고 TF 시스템에 물어보면(Lookup),         
TF 시스템이 즉시 "Lidar가 (0.5, 0)에 있으니까, 그 벽은 base_link 기준 (2.5, 0)이야!"라고 좌표 변환을 계산해줍니다.      
### TF 핵심 용어 
Frame (프레임): 각 부품의 고유한 "좌표계" 이름입니다. (예: base_link, lidar_link, odom, map)      
Transform (변환): 한 Frame에서 다른 Frame으로 가는 "변환 정보"입니다. (위치(x,y,z) + 회전(Quaternion))       
Broadcaster (발행자): robot_state_publisher처럼 TF 정보를 방송하는 노드입니다.       
Listener (수신자): Nav2처럼 TF 정보를 받아서 계산을 요청하는 노드입니다.      
### The ROS Transform System (TF)
#### Broadcasting static TFs
```
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_frame child_frame
```
의 기본 구조를 갖는다.     
<img width="984" height="578" alt="image" src="https://github.com/user-attachments/assets/a97bf415-5817-485d-a391-53456706abde" />      

이 이미지를 구현해보자.   
```
ros2 run tf2_ros static_transform_publisher 2 1 0 0.705 0 0 world robot_1
```
world의 좌표에서 robot_1좌표까지의 특성을 적어서 tf2를 작동시켜준다. 이후 robo_1좌표에서 robot_2좌표도 구현해준다.   
```
ros2 run tf2_ros static_transform_publisher 1 0 0 0 0 0 robot_1 robot_2
```
이후 rviz2를 켜준다. 
```
ros2 run rviz2 rviz2 
```
이후 add에서 tf를 add해주고 fixed Frame을 world로 바꿔주면 다음과 같이 뜬다.    
<img width="1793" height="1022" alt="image" src="https://github.com/user-attachments/assets/20f9650b-f476-4b10-805a-a6ea7ab7c13c" />          

<img width="1793" height="1022" alt="image" src="https://github.com/user-attachments/assets/2653f3ea-8ea8-493c-b367-96c8c1e8fd66" />       

yaw값을 90도 1.57rad으로 바꾸어 보면 다음과 같이 바뀌는 것을 확인할 수 있다.     
<img width="1793" height="1022" alt="image" src="https://github.com/user-attachments/assets/d08160e5-22b2-4315-8ed9-51391bb89418" />       

#### Broadcasting dynamic TFs
<img width="1350" height="748" alt="image" src="https://github.com/user-attachments/assets/966b96db-a60b-4335-83d3-8288f9dce1a6" />       

<img width="1350" height="748" alt="image" src="https://github.com/user-attachments/assets/fc2e6ad8-7954-462e-b24e-ba3216f5c142" />        

### TF를 만드는 2가지 재료
1. URDF     
런치 파일이 .xacro 파일을 URDF 텍스트로 변환해서, robot_state_publisher 노드를 실행할 때 **파라미터(robot_description)**로 전달해 줍니다.      
이것은 로봇의 "뼈대"가 어떻게 생겼는지 알려주는 정적인(static) 정보입니다. (예: "팔꿈치는 어깨에서 30cm 아래에 있다.")      

2. 현재 관절 각도(/joint_states 토픽)      
robot_state_publisher는 /joint_states라는 토픽을 **구독(subscribe)**합니다.
다른 노드(예: Gazebo 플러그인, 실제 로봇 드라이버, RViz의 슬라이더)가 이 토픽으로 "현재 팔꿈치가 30도 굽혀졌다!" 같은 동적인(dynamic) 정보를 계속 보냅니다.     

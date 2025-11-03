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

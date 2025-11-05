## Easy SLAM with ROS using slam_toolbox
이번에는 ros의 slam_toobox를 이용하여 로봇 주변 환경의 맵을 생성할 것이다.     
먼저 로봇에 부착된 프래임을 base_link라고 한다.     
또한 다른 참조가 없는 고정 프레임을 Odom이라고 한다. 이 프레임은 world의 원점이다.     
odem으로부터의 base_link는 부정확할 수 있지만 smooth하다. 
<img width="1280" height="706" alt="image" src="https://github.com/user-attachments/assets/eec52667-4c55-4873-bda9-b9c4a4af3a5d" />

odeom은 단기적으로는 정확하지만, 로봇이 계속 움직이면 반드시 실제 세계와 오차가 벌어진다. 이러한 오차를 바로 잡기 위해서 lider나 카메라같은 센서가 필요하다.     
lidar/SLAM이 map좌표계를 만들고 odom의 오차를 보정정보를 준다. 이러한 보정정보가 map-->odom의 TF(좌표 변환)이다.      
<img width="1280" height="706" alt="image" src="https://github.com/user-attachments/assets/dedea0ef-ac9e-41aa-8f00-102aae24743a" />      

<img width="1280" height="706" alt="image" src="https://github.com/user-attachments/assets/6e8bb494-e232-4761-ae53-7f0758223f4d" />      

다음의 과정을 따르게 된다. 이것이 map to odom transform이다.       
<img width="1280" height="706" alt="image" src="https://github.com/user-attachments/assets/82210f60-c7b0-4d34-ae6f-01d3aab09bdb" />    

odom (프레임): 표류하는(drifting) 로봇의 상대적인 출발점.       
map (프레임): 벽을 기준으로 한 고정된(fixed) 절대적인 세계 지도.       
/odom (토픽): 로봇의 현재 속도와 (표류 중인) 위치 정보.      
/map (토픽): 지도 그림 데이터 (벽, 빈 공간).        

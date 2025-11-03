## URDF
### URDF syntax
기본 구조    
```
<?xml version="1.0"?>    
<robot name="robot">
  <link>
  </link>

  <joint>
  </joint>

  <link>
  </link>

  ...

</robot>
```
링크는 하나의 이름을 갖고 3개의 특성을 갖는다. 
```
<link name="arm_link">
  <visual>
  </visual>

  <collision>
  </vollision>

  <inertial>
  </inertial>
</link>
```
먼저 visual이다.
```
<visual>
  <geometry>
  <origin>
  <material>
</visual>
```
```
geometry: 실제모양(하위테그--> <box size="..."/>: 직육면체, <cylinder radius="..." length="..."/>: 원기둥, <sphere radius="..."/>: 구, <mesh filename="..."/>: (가장 많이 사용됨) 3D 모델링 파일(.stl, .dae)을 불러옵니다.)      
origin: 위에서 정의한 <geometry>(모양)를 이 링크(부품)의 기준점(0,0,0)에서 얼마나 떨어뜨리고 회전시킬지 정의한다.(하위속성--> xyz="...": x, y, z 축으로 이동할 거리 (m), rpy="...": Roll, Pitch, Yaw (오일러 각)로 회전할 각도 (rad).)       
material: 이 부품의 색상이나 질감 정의(하위테그--> <color rgba="..."/>: R G B Alpha (빨강, 초록, 파랑, 투명도) 값을 0~1 사이로 지정합니다. (예: rgba="1 0 0 1" = 불투명한 빨간색), <texture filename="..."/>: 색상 대신 이미지 텍스처 파일을 입힐 때 사용합니다.)
``` 
다음은 collision이다.
```
<collision>
  <geometry>
  <origin>
</collision>
```
```
collision은 로봇이 물리엔진 안에서 실제로 부딪히는 모양을 정의한 블록이다. <geometry>와 <origin>은 위의 내용과 같다.
```
다음은 inertial이다. 
```
<inertial>
  <mass>
  <origin>
  <inertia>
</inertial>
```
inertial은 물리 시뮬레이터가 이 부품의 물리 법칙을 계산하는데 필요한 핵심 속성을 정의하는 블록     
```
mass: 이 부품의 **무게(질량)**를 정의합니다. (속성--> value="...": 질량 값을 킬로그램(kg) 단위로 지정합니다. (예: value="2.5"))      
origin: 이 부품의 **무게 중심(Center of Mass, CoM)**이 링크의 기준점(0,0,0)에서 얼마나 떨어져 있는지 그 위치를 정의합니다. (속성--> xyz="...": 기준점으로부터 무게 중심까지의 x, y, z 거리 (m)., rpy="...": (보통 0) 무게 중심의 좌표축 방향 (rad).)       
inertia: 이 부품이 회전에 대해 얼마나 저항하는지를 정의하는 "회전 질량" 값입니다.(속성--> ixx, iyy, izz: x, y, z축을 중심으로 회전할 때의 저항값. (값이 클수록 그 축으로 돌리기 어렵습니다.), ixy, ixz, iyz: 관성곱. 물체가 비대칭일 때 축들이 서로 어떻게 영향을 주는지 나타내는 값입니다. (복잡한 개념으로, 단순한 상자나 원기둥은 0입니다.))
```
다음은 joint이다. joint는 이름과 타입을 적어준다.    
```
<joint name="arm_joint" type="revolute">
  <parent link="slider_link"/>
  <child link="arm_link"/>
  <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
  <axis xyz="0 -1 0"/>
  <limit lower="0" upper="${pi/2}" velocity="100" effort="100"/>
</joint>
```
```
type: revolute(회전 운동), prismatic (직선 운동), fixed (완전 고정), continuous (무한 회전) 등이 있습니다.     
<parent link=...> : 관절이 연결되는 부모(고정측) 부품입니다.     
<child link=...> : 이 관절에 의해 움직이는 자식(운동측) 부품입니다.      
<origin xyz="0.25 0 0.15" rpy="0 0 0"/>: 이 관절(회전축의 중심)이 **부모 링크(slider_link)의 기준점(0,0,0)**으로부터 얼마나 떨어져 있는지를 나타냅니다.
<axis xyz="0 -1 0"/>: revolute (회전) 타입 관절의 **"회전축 방향"**을 정의합니다.       
<limit lower="0" upper="${pi/2}" velocity="100" effort="100"/>: 관절의 운동 범위와 한계를 설정합니다.       
lower="0": 최소 가동 범위 (0 라디안).     
upper="${pi/2}": 최대 가동 범위 (π/2 라디안, 즉 90도).       
velocity="100": 이 관절이 움직일 수 있는 최대 속도 (초당 100 라디안).       
effort="100": 이 관절이 버티거나 낼 수 있는 최대 힘(토크) (100 뉴턴미터).      
```
     

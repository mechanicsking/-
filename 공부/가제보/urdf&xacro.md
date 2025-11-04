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
### Using xacro to make things easier
xacro 파일을 만들때에는 로봇태그에 다음의 작은 추가 비트를 추가한다. 
```
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
```
<img width="1278" height="713" alt="image" src="https://github.com/user-attachments/assets/e9680c7b-4c3d-471a-acc1-39065648a135" />     

xacro는 다음과 같이 사용된다. 이렇게 사용한다면 launch file을 한번만 작성하고 원하는 만큼 프로젝트에서 재사용할 수 있게 된다.     

<img width="1278" height="713" alt="image" src="https://github.com/user-attachments/assets/0811ff07-6cd0-4267-bec9-2ec0e1cfdc6a" />       

이런식으로 재사용이 쉽게 분리해두고 xacro를 통해 결합해주면 된다.   
xacro의 또다른 특성은 코드반복을 피하는 것이다. urdf 파일을 단순화하고 반복을 피하고 실수를 피하는데 도움이 되는 많은 방법이있다.    
#### 1. <xacro:property> (변수)
프로그래밍에서의 변수와 같은 개념,, 자주 사용하는 값에 라벨을 붙여서 관리하는 기능이다.     
역할: 로봇의 크기, 관절 한계, 부품 이름 등을 변수로 저장
예시: 로봇의 다리 길이를 0.3 미터로 정하고 싶을 때:
```xml
<xacro:property name="leg_length" value="0.3" />

<link name="thigh_link">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="${leg_length}" />
    </geometry>
  </visual>
</link>
```
#### 2. <xacro:include> (파일 분리 / 가져오기)
이건 파이썬의 import나 C언어의 #include와 같다. 다른 .xacro 파일을 현재 파일로 "가져오는" 기능     
역할: 거대한 로봇 모델을 기능별(몸통, 다리, 센서)로 깔끔하게 나눌 수 있다.     
예시: go2_robot.xacro (메인 파일) 안에서:     
```xml
<xacro:include filename="legs.xacro" />

<xacro:include filename="sensors.xacro" />

<xacro:include filename="gazebo_plugins.xacro" />
```
#### 3. <xacro:macro> (함수 / 복사-붙여넣기 방지)
이건 프로그래밍의 **함수(Function)**와 같다. 반복되는 코드 뭉치를 "블록"으로 만드는 기능이다.    
역할: 4족 로봇의 "다리"처럼, 똑같이 생겼지만 이름(prefix)만 다른 부품을 만들 때 복사-붙여넣기(Copy-Paste)를 방지해 준다.     
```xml
<xacro:macro name="robot_leg" params="prefix">

    <link name="${prefix}_thigh_link"> ... </link>
    <joint name="${prefix}_thigh_joint"> ... </joint>
    </xacro:macro>

<xacro:robot_leg prefix="front_left" />
<xacro:robot_leg prefix="front_right" />
<xacro:robot_leg prefix="back_left" />
<xacro:robot_leg prefix="back_right" />
```
이제 example urdf를 확인해보자. 
https://github.com/joshnewans/urdf_example  를 git clone을하여 따라가보자    

<img width="1568" height="930" alt="image" src="https://github.com/user-attachments/assets/d5d0efe1-07fd-4696-9b14-8278d8470dc3" />     

다섯개의 링크와 4개의 joint가 있는 것을 확인할 수 잇다. 


<img width="1278" height="713" alt="image" src="https://github.com/user-attachments/assets/26fe4a9e-38e9-4081-a347-89c55fb88170" />      

example_robot.urdf.xacro 파일을 확인해보면 
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot">
    <xacro:include filename="example_include.xacro" />
```
example_include.xacro를 include하고 있다. 따라서 해당 파일을 살펴보면 
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >

    <!-- This file is not a robot in and of itself, it just contains some useful tags that could be included in any robot -->



    <!-- Specify some colours -->

    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>

    <material name="orange">
        <color rgba="1 0.3 0.1 1"/>
    </material>

    <material name="blue">
        <color rgba="0.2 0.2 1 1"/>
    </material>



    <!-- Specify some standard inertial calculations https://en.wikipedia.org/wiki/List_of_moments_of_inertia -->
    <!-- These make use of xacro's mathematical functionality -->

    <xacro:macro name="inertial_sphere" params="mass radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(2/5) * mass * (radius*radius)}" ixy="0.0" ixz="0.0"
                    iyy="${(2/5) * mass * (radius*radius)}" iyz="0.0"
                    izz="${(2/5) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>  


    <xacro:macro name="inertial_box" params="mass x y z *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                    izz="${(1/12) * mass * (x*x+y*y)}" />
        </inertial>
    </xacro:macro>


    <xacro:macro name="inertial_cylinder" params="mass length radius *origin">
        <inertial>
            <xacro:insert_block name="origin"/>
            <mass value="${mass}" />
            <inertia ixx="${(1/12) * mass * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                    iyy="${(1/12) * mass * (3*radius*radius + length*length)}" iyz="0.0"
                    izz="${(1/2) * mass * (radius*radius)}" />
        </inertial>
    </xacro:macro>
```
먼저 흰색 주황색 파란색을 설정하고, 일부 macro를 설정한다. inertial_sphere과 inertial_box, inertial_cylinder를 설정해준다. 이는 다음에 사용되진다. 그리고 다시 example_robot.urdf.xacro 파일로 돌아와서 확인해본다. 
```xacro
    <link name="world"></link>
    <joint name="base_joint" type="fixed">
        <origin xyz="1.5 1.0 0" rpy="0 0 0"/>
        <parent link="world"/>
        <child link="base_link"/>        
    </joint>

    <link name="base_link">
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="2.5 1.5 0.1" />
            </geometry>
            <material name="green">
                <color rgba="0.2 1 0.2 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="2.5 1.5 0.1" />
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <mass value="12" />
            <inertia ixx="2.26" ixy="0.0" ixz="0.0" iyy="6.26" iyz="0.0" izz="8.5" />
        </inertial>
    </link>

```
world링크와 base_link를 이어주는 조인트가 정의되어있다. 다음은 slidr_link와 이어주는 slider_joint이다. 
```xacro
    <joint name="slider_joint" type="prismatic">
        <origin xyz="-1.25 0 0.1" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="slider_link"/>
        <axis xyz="1 0 0"/>
        <limit lower="0" upper="2" velocity="100" effort="100"/> 
    </joint>

    <link name="slider_link">
        <visual>
            <origin xyz="0 0 0.075" rpy="0 0 0"/>
            <geometry>
                <box size="0.5 0.25 0.15" />
            </geometry>
            <material name="blue" />
        </visual>
        <collision>
            <origin xyz="0 0 0.075" rpy="0 0 0"/>
            <geometry>
                <box size="0.5 0.25 0.15" />
            </geometry>
        </collision>
        <xacro:inertial_box mass="0.5" x="0.5" y="0.25" z="0.15">
            <origin xyz="0 0 0.075" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>
```
이 joint에서 중요한건 type을 잘 살피는 것과 limit부분이다. 그리고 자세히 볼 부분이 slider_link의 material부분이다. 우리는 파란색으로 하고 싶고, 이미 만들어둔 xacro파일을 메크로 했기때문에 다음과 같이 "blue"라고 적어도 작동한다. 또한 inertial부분에서 xacro를 사용하는 것을 확인해볼 수 있다. 이미 우리는 인수를 주면 계산을 하여 inertial을 계산할 수 있는 메크로를 만들었기 때문에 간편하게 값을 받을 수 잇는 것을 확인 할 수 있다. 그리고 여기서의 <origin xyz="0 0 0.075" rpy="0 0 0"/>는 메크로 파일의 <xacro:insert_block name="origin"/>로 들어가게 된다.     
다음은 arm_link와 연결하는 arm_joint이다.   
```xacro
    <joint name="arm_joint" type="revolute">
        <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
        <parent link="slider_link"/>
        <child link="arm_link"/>
        <axis xyz="0 -1 0"/>
        <limit lower="0" upper="${pi/2}" velocity="100" effort="100"/> 
    </joint>

    <xacro:property name="arm_length" value="1" />
    <xacro:property name="arm_radius" value="0.1" />
    <link name="arm_link">
        <visual>
            <origin xyz="${arm_length/2} 0 0" rpy="0 ${pi/2} 0"/>
            <geometry>                
                <cylinder length="${arm_length}" radius="${arm_radius}" />
            </geometry>
            <material name="orange" />
        </visual>
        <collision>
            <origin xyz="${arm_length/2} 0 0" rpy="0 ${pi/2} 0"/>
            <geometry>
                <cylinder length="${arm_length}" radius="${arm_radius}" />
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="1.0" length="${arm_length}" radius="${arm_radius}">
            <origin xyz="${arm_length/2} 0 0" rpy="0 ${pi/2} 0"/>
        </xacro:inertial_cylinder>
    </link>
```
여기에서는 xacro의 property기능을 사용해 arm_length와 arm_radius를 미리 지정해준다.     
다음은 camera_link와 camera_joint이다.     
```xacro
    <joint name="camera_joint" type="fixed">
        <origin xyz="${arm_length} 0 ${arm_radius + 0.075}" rpy="0 0 0"/>
        <parent link="arm_link"/>
        <child link="camera_link"/>        
    </joint>

    <link name="camera_link">
        <visual>
            <origin xyz="-0.03 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="0.06 0.15 0.15" />
            </geometry>
            <material name="white" />
        </visual>
        <visual>
            <origin xyz="0.03 0 0" rpy="0 ${pi/2} 0"/>
            <geometry>
                <cylinder length="0.06" radius="0.04" />
            </geometry>
            <material name="blue" />
        </visual>
        <collision>
            <origin xyz="0.0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="0.12 0.15 0.15" />
            </geometry>
        </collision>
        <xacro:inertial_box mass="0.1" x="0.12" y="0.15" z="0.15">
            <origin xyz="0.0 0 0" rpy="0 0 0"/>
        </xacro:inertial_box>
    </link>


    <xacro:include filename="example_gazebo.xacro" />


</robot>
```
두개의 visual로 되어있으며 collision은 1개의 박스로 만들어주었다. 

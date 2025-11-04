## Simulating Robots with Gazebo and ROS
world를 다시 되돌리고 싶다면 edit에서 reset world or ctrl+R    
물체에 우클릭을 하고 apply Force and Torqe를 한다면 힘도 줄수 있다.     
<img width="1840" height="1078" alt="image" src="https://github.com/user-attachments/assets/aecb3ebf-8d7a-4078-a515-873e600aaebc" />      

이것은 단지 물리 시뮬레이터이다. 이제 부터 ros를 사용하여 로봇을 이 시뮬레이션 환경으로 가져오는 방법을 보자. 가장 먼저 이해할 것은 가제보 모델의 구조이다. 로봇을 위한 URDF를 만들어보았고 가제보도 SDF라는 유사한 형식을 사용한다. URDF와 비슷하지만 살짝 다른다. 가제보에서는 urdf를 sdf로 자동으로 변환할 수 잇는 도구를 제공하기 때문에 두개의 다른 파일을 작성할 필요가 없다. sdf는 시뮬레이션 되는 world와 그 world내부의 모델을 설명하는데 사용된다. 아까 보았던 seesaw.world파일은 sdf파일이었지만 그 안의 각각의 작은 큐브들도 각각의 sdf파일이 될 수있다. 이는 다른 world에서 모델을 재사용할 수 있게 해준다.   
또 이해해야할 다른 점은 gazebo가 ros와 같은 외부의 무언가와 상호 작용할 때마다 플러그인이라는 것을 사용해야한다. 이것은 특정시간에 가제보에 실행하도록 지시할 수 잇는 특정 작업을 수행하는 추가 코드이다.     
<img width="1278" height="713" alt="image" src="https://github.com/user-attachments/assets/983c1d74-aa85-4e85-bf2c-08c246732894" />     

이 시스템은 크게 두 부분으로 나뉩니다.     
Gazebo (아래쪽 박스): "실제" 물리 법칙이 적용되는 가상 세계입니다.     
robot_state_publisher (위쪽 노드): 가상 세계의 데이터를 받아서 ROS 2 시스템 전체가 알아들을 수 있는 **좌표 정보(TF)**로 "번역"하는 계산기입니다.     
#### 1. 가제보 (Gazebo) - "시뮬레이션 세계"
이 박스는 시뮬레이터 그 자체입니다. 이 안에서 로봇이 물리적으로 존재하고 움직입니다. 이 로봇이 ROS와 소통할 수 있는 이유는 플러그인(Plugin) 덕분입니다.

* Joint controller plugin (근육 💪):
  control input (제어 명령) 토픽을 구독합니다.
  명령을 받으면 Gazebo 속 로봇의 관절을 물리적으로 움직입니다.
* Joint state publisher plugin (신경 🧠):
  Gazebo 속 로봇의 관절이 실제로 몇 도인지 실시간으로 읽습니다.
  이 정보를 /joint_states라는 토픽으로 **발행(Publish)**합니다. (이 정보가 위쪽 robot_state_publisher의 재료가 됩니다.)
* Sensor Plugin (감각 👁️):
  Lidar나 카메라, IMU 같은 센서를 시뮬레이션합니다.
  센서 데이터를 /my_sensor 같은 토픽으로 **발행(Publish)**합니다.

#### 2. robot_state_publisher - "로봇 상태 계산기" 📐
이 노드는 Gazebo와 별개로 실행되는 ROS 2 노드입니다. 이 노드가 바로 URDF와 TF를 연결하는 핵심입니다.     

* URDF File (설계도):
  로봇의 "뼈대" 정보입니다. (예: "팔 길이는 30cm")
  이 정보를 /robot_description이라는 토픽으로 **재발행(Copy)**하여, 다른 노드(예: RViz, Spawner)들이 "설계도 사본"을 쉽게 볼 수 있게 해줍니다.
* /joint_states (실시간 각도):
  Gazebo의 Joint state publisher plugin("신경")이 보낸 "현재 관절 각도" 정보입니다. (예: "팔꿈치가 30도 굽혀짐")

robot_state_publisher는 이 설계도와 실시간 각도를 1초에 수십 번씩 조합(계산)하여,     
**최종 결과(Output)**인 **Joint transforms (최종 3D 자세)**를 계산해 **TF System (즉, /tf 토픽)**으로 쉴 새 없이 발행합니다.      

#### 3. 스포너 (Spawner Script) - "로봇 창조자" 🪄�
Spawner Script는 robot_state_publisher가 발행한 /robot_description (설계도 사본)을 구독한 뒤, Gazebo에게 "이 설계도대로 로봇을 시뮬레이션에 만들어 줘!"라고 명령하여 로봇을 세상에 스폰(spawn)시킵니다.

🌟 전체 흐름 요약
1. robot_state_publisher가 **URDF**를 읽고 /robot_description 토픽으로 발행합니다.
2. Spawner Script가 /robot_description을 듣고 Gazebo에 로봇을 생성합니다.
3. Gazebo의 Joint state publisher plugin이 로봇의 관절 각도를 /joint_states 토픽으로 발행합니다.
4. robot_state_publisher가 /joint_states를 구독하고, URDF와 조합하여 최종 자세를 **TF System**으로 발행합니다.
5. RViz 같은 다른 노드들은 TF System을 보고 로봇의 현재 자세를 그립니다.

### Example
우리는 urdf chapter에서 만든 것에서 urdf를 업그레이드하여 시뮬레이션된 가상환경에 넣을 것이다.   우리는 ros2 통합으로 가제보를 실행할 것이기 때문에 다음과 같은 명령어로 가제보를 열어줄거다.    
```
ros2 launch gazebo_ros gazebo.launch.py 
```
이렇게 하면 gazebo가 실행되고 비어있는 공간으로 시작한다. 그리고 우리가 하려는 것은 로봇을 생성하는 것이다. urdf 파일을 sdf파일로 변환하여 가제보 world에 생성하는 코드도 제공되어있다.  
```
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot
```
이렇게 하면/robot_description라는 토픽을 구독하여 그 토픽에 있는 로봇 설계도를 받아서 sdf로 바꿔 가제보 월드로 로봇을 생성해준다. 그 토픽은 미리 만든 lauch를 이용해준다.    
```
ros2 launch urdf_example rsp.launch.py 
```
이러면 가제보에 my_bot이라는 이름으로 로봇이 생성된다.    
<img width="1716" height="1158" alt="image" src="https://github.com/user-attachments/assets/15578350-dca3-456b-a279-f6e69e239152" />      

색이 이상한 걸 확인할 수 있다.    
이를 한번에 할 수 있게 런치 파일을 만들었다.       
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'urdf_example'
    file_subpath = 'description/example_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')






    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
```
런치파일을 실행하면 다음과 같이 작동한다. 
```
ros2 launch urdf_example rsp_sim.launch.py 
```

우리는 로봇에 기능을 추가해 줄것이다. 따라서 example_robot.urdf.xacro파일에 </robot> 전에 <xacro:include filename="example_gazebo.xacro" />를 추가해준다. 따라서 이를 통해 여기에 있는 기능들을 include해줄 것이다.    
example_gazebo.xacro파일을 <gazebo>태그를 이용하여 다음과 같이 만들어줄것이다. <gazebo> 태그는 Gazebo 시뮬레이터 전용 설정을 URDF/Xacro 파일 안에 추가하기 위한 태그이다. ROS의 다른 도구들(예: RViz나 robot_state_publisher)은 이 <gazebo> 태그를 완전히 무시한다. 오직 Gazebo 시뮬레이터만 이 태그를 읽어서 사용한다.     
#### <gazebo> 태그의 핵심 사용처
##### 1. 플러그인(Plugin) 로드 (가장 중요)
이것이 <gazebo> 태그의 가장 중요한 용도이다. Gazebo 안에서 ROS 2 토픽과 통신하거나 로봇을 움직이게 하는 "소프트웨어"를 로드한다.     
예시: 
```xacro
<gazebo reference="base_link">
  <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <command_topic>cmd_vel</command_topic>
    </plugin>
</gazebo>
```
##### 2. 센서(Sensor) 시뮬레이션 정의
Lidar, 카메라, IMU 같은 센서가 Gazebo 안에서 어떻게 작동할지(데이터 발행 주기, 노이즈 등) 정의한다. 
```xacro
<gazebo reference="lidar_link">
  <sensor type="ray" name="my_lidar_sensor">
    <update_rate>10</update_rate> <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
      </plugin>
  </sensor>
</gazebo>
```
##### 3. 물리 속성(Physics) 및 색상 정의
URDF의 <collision>이나 <visual>에서 설정할 수 없는, Gazebo 전용 물리 값(마찰력, 반발력)이나 색상을 정의한다. 
```xacro
<gazebo reference="wheel_link">
  <mu1>1.0</mu1> <mu2>1.0</mu2> <material>Gazebo/Red</material>
</gazebo>
```
example_gazebo.xacro로 다시 돌아온다. 
```xacro
    <gazebo reference="base_link">
        <material>Gazebo/Green</material>
    </gazebo>

    <gazebo reference="slider_link">
        <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="arm_link">
        <material>Gazebo/Orange</material>
    </gazebo>
```
원래대로 한다면 흰색이 뜨게 되는데 가제보에서 제공하는 색상을 사용하여 색이 잘 나올 수 잇게 해준다.       
<img width="1866" height="1056" alt="image" src="https://github.com/user-attachments/assets/1fa2f6b4-acfd-4905-ac2b-4d78dcfe62bf" />       
색이 잘 나오는 것을 확인해 볼 수 있다.    
아직 조인트의 상태를 publish하지 않았기에 움직일 순 없다. 그다음 이 비트를 추가한다.     
```xacro
    <gazebo>
        <plugin name="gazebo_ros_joint_state_publisher"
            filename="libgazebo_ros_joint_state_publisher.so">
            <update_rate>20</update_rate>
            <joint_name>slider_joint</joint_name>
            <joint_name>arm_joint</joint_name>
        </plugin>
    </gazebo>
```
이 파트는 joint의 state를 publish하는 파트이다. 이 플러그인이 실행되면, /joint_states (기본 토픽 이름)라는 토픽에 slider_joint와 arm_joint의 현재 각도(position) 정보를 1초에 20번씩 발행(publish)한다. 그러면 robot_state_publisher 노드가 이 /joint_states 토픽을 구독(subscribe)하여 최종 TF를 계산할 수 있게 된다. 이 플러그인은 gazebo_ros 패키지가 기본으로 제공하는 플러그인이다.    
우리가 지금까지 한것은 joint controller plugin을 사용할 수 있게 해주었다.       
<img width="1276" height="722" alt="image" src="https://github.com/user-attachments/assets/ec5dfa6e-7b25-4995-8c38-016f43747b98" />     

다음으로 할건 joint_pose_trajectory 플러그인을 가져올것이다. 
```xacro
    <gazebo>
        <plugin name="gazebo_ros_joint_pose_trajectory"
            filename="libgazebo_ros_joint_pose_trajectory.so">
            <update_rate>2</update_rate>
        </plugin>
    </gazebo>
```
여기서 가진 유일한 매개변수는 업데이트 속도이다. 조인트들이 너무 쉽게 움직이기 때문에 우리는 arm_joint와 slider_joint 조인트에 <dynamics damping="10.0" friction="10.0"/>를 추가하여 현실과 같이 만들어준다.     
이제 토픽을 publish하여 각도를 조절해보자.   
```
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory  '{header: {frame_id: world}, joint_names: [slider_joint, arm_joint], points: [  {positions: {0.8,0.6}} ]}'
```
<img width="1786" height="844" alt="image" src="https://github.com/user-attachments/assets/3b632d40-2a08-4a35-8f62-880c33b90e38" />       

잘작동한다. 이제 ros를 이용해 가제보 시뮬레이터를 움직일 수 있고 물리 법칙이 적용하여 다시 ros에 publish한다. 다음으로 할 것은 센서를 시뮬레이션하는 것이다. 가장 흥미로운 부분중 하나이다. 일단 카메라를 설치해서 world를 볼것이기에 world를 먼저 꾸며보자. insert탭을 통해 모델들을 추가한다.  
우리가 ros에서 카메라를 사용할 때에는 광학조인트라는 추가 링크를 넣어야한다.      
```xacro
    <joint name="camera_optical_joint" type="fixed">
        <origin xyz="0 0 0" rpy="-1.571 0 -1.571" />
        <parent link="camera_link" />
        <child link="camera_link_optical" />
    </joint>

    <link name="camera_link_optical"></link>
```
그리고 마지막으로 카메라를 시뮬레이션하는데 필요한 태그를 가지고 온다.       
```xacro
    <gazebo reference="camera_link">
        <sensor type="camera" name="my_camera">
            <update_rate>20</update_rate>
            <visualize>true</visualize>
            <camera name="cam">
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8B8G8</format>
                </image>
                <clip>
                    <near>0.02</near>
                    <far>300</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
            </camera>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <frame_name>camera_link_optical</frame_name>
                <min_depth>0.1</min_depth>
                <max_depth>500</max_depth>
            </plugin>
        </sensor>
    </gazebo>
```
이제 gazebo에서 볼 수 있는 것은 카메라가 보고 있는 것의 미리보기를 확인할 수 있다.     
 <img width="1786" height="844" alt="image" src="https://github.com/user-attachments/assets/f1857b25-b516-4ea8-a9d7-60b0be79970e" />        

 rviz2에서도 image를 구독하면 카메라로 찍히는 것을 rviz로 가져올 수 있다.       
<img width="1675" height="1023" alt="image" src="https://github.com/user-attachments/assets/a2c0dd57-29d2-44af-aecc-354d10de853e" />        

gazebo에서 시뮬레이션되고 이미지가 ros로 전달되고 rviz에 표시할 수 있으며, 해당 topic에 publish되고 있다. 이번에는 depth카메라를 가져와보자. sensor type을 depth로 변경한다.     
depth카메라를 사용할 때는 작은 미리보기가 나오지 않는다.      

<img width="1675" height="1023" alt="image" src="https://github.com/user-attachments/assets/38695dca-ec63-4cf0-a521-6c85badb41b1" />      

여기서 확인해 볼 수 있고 포인트 클라우드를 추가할 수도 있다. 포인트 클라우드를 확인하면 다음과 같이 확인해 볼 수 있다.     
<img width="1675" height="1023" alt="image" src="https://github.com/user-attachments/assets/787a5e69-ef69-45de-b11c-47374be6f63f" />        

가제보내에서 나무를 움직이면 rviz에서도 움직이는 것을 볼 수 있다.     
<img width="3028" height="990" alt="image" src="https://github.com/user-attachments/assets/f11245b9-7c08-42d1-a97c-89ef9b050736" />     

ros2 topic echo /clock을 통해 시간을 가져올 수 있다.     
<img width="1368" height="861" alt="image" src="https://github.com/user-attachments/assets/893ea71c-42d6-448c-92b2-ab28606af632" />      
이 시간을 보고 다른 노드들이 시간을 유지할 수 있다.     
주의점!!    
가제보는 실제로 하나의 프로그램이 아니랄 두개의 프로그램이다. 서버와 클라이언트가 있다. 한개에서 충돌이 발생할 수 있고, 둘다 충돌이 발생할 수 있다.     

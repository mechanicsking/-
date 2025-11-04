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
<img width="1568" height="930" alt="image" src="https://github.com/user-attachments/assets/094c949d-387d-4029-b38e-7a05f9c7860c" />        

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
<img width="1568" height="930" alt="image" src="https://github.com/user-attachments/assets/1de774bd-f5a9-4f64-ad19-0c2511ef4e41" />        

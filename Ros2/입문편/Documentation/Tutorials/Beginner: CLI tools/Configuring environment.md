(25.07.26작성)
# Configuring environment 환경구성하기
* 목표: ros2환경 구성하기

## Background(배경지식)
ros2는 쉘 환경을 사용하여 workspace를 결합하는 개념에 의존한다. "Workspace"란 ros2로 개발중인 시스템의 위치를 나타내는 ros용어이다.    
core ros2 workspace를 underlay라고 부른다. local workspaces를 overlays라고 부른다. ros2를 개발 할 때는 일반적으로 여러 workspaces들이 동시에 활성화 된다.      
workspaces를 결합하면 다른버전의 ros2나 패키지 세트에 대한 개발이 쉬워진다. 다양한 ros2를 같은 컴퓨터에 설치하고 이를 전환할 수 잇다.     
이 작업은 매번 새로운 shell을 열 때마다 설정 파일을 sourcing하거나 shell 시작 스크립트에 source command를 추가하여 수행한다.     
setup file을 sourcing하지 않는다면, ros2에 access하거나 ros2 패키지를 찾을 수 없다. 다시말해 ros2를 사용하지 못한다.    
## Tasks
### 1. source the setup files
다음의  명령어를 매번 새로운 shell을 오픈할 때마다 작동시켜야한다.    
```
$source /opt/ros/humble/setup.bash
```
만약 bash를 사용하지 않고 다른 shell을 사용한다면 .bash를 수정해야하며, 수정한 것은 다음과 같다. setup.bash, setup.sh, setup.zsh     
### 2. Add sourcing to your shell startup script
새 셸을 열 때마다 설정 파일을 소스화할 필요가 없다면(작업 1 건너뛰기), 셸 시작 스크립트에 명령을 추가할 수 있다.    
```
$echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
이를 취소하려면 시스템의 셸 시작 스크립트를 찾아 첨부된 소스 명령을 제거해야한다.   
### 3. Check environment variables
ros2 setup files를 소싱하면 ros2를 운영하는데 필요한 여러 환경변수가 설정된다. ROS 2 패키지를 찾거나 사용하는 데 문제가 있는 경우 다음 명령을 사용하여 환경이 올바르게 설정되어 있는지 확인해야한다.     
```
$printenv | grep -i ROS
```
ros_distro와 ros_verion을 확인해본다.     
ROS_VERSION=2      
ROS_PYTHON_VERSION=3    
ROS_DISTRO=humble    
위와 같이 나온다면 이 튜토리얼에 필요한 버전이다. 그렇지 않는다면 ROS 2 package installation section에서 다시 installation을 해야한다.     
* 3.1 The ROS_DOMAIN_ID variable
  domain ID를 확인해보자. ros2 노드 그룹의 unique integer를 결정한 후 다음의 명령으로 환경변수를 설정할 수 있다.    
  ```
  $export ROS_DOMAIN_ID=<your_domain_id>
  ```
  셀 세션 간에 이 설정을 유지하려면 셀 시작 스크립트에 명령을 추가 할 수 있다.
   ```
  $echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
  ```
* 3.2 The ROS_LOCALHOST_ONLY variable
  

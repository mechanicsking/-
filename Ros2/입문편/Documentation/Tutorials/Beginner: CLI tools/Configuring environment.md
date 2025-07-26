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

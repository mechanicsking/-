## 2-3 bashrc 설정
현재 shell 확인

    ~$ echo $SEHLL
    /bin/bash
shell이란?-->셸은 키보드 명령을 받아 운영 체제에 전달하여 실행하는 프로그램

우분투는 기본으로 bash이다

.bashrc란 터미널을 실행했을때 가장 먼저 실행하는 설정들을 보관하고 있는 파일

어떤 에디터든 .bashrc를 열 수 있다. vscode으로 열어보자

    ~$ code ~/.bashrc
열어서 마지막에 다음을 추가

    echo "ROS2 humble is activated!"
    source /opt/ros/humble/setup.bash
source명령을 썻으니 그것을 명시해주고자 echo(=파이썬의 print)를 사용해줌

그 후 source를 사용하라고 명령, ctrl+s를 사용해 저장
## 2-4 bashrc에서 alias 설정하기
alias라는 명령을 통해서 내가 원하는 명령을 만들수 잇다.

    alias command_name="values"
command_name이라는 명령을 만드는데 그 내용은 values이다. 

주의:name과 value사이의 등호(=)는 꼭 띄어쓰기 없이 붙여 써야 한다.

alias 지정 연습

    alias alias_test="echo \"Alias test\""
echo "~~" 인데 ""가 너무 많기 때문에 안에 있는 문자열이라는 것을 표현하기 위해 \"~\"로 표현

alias라는 명령을 통해 alias_test라는 명령을 만들것이고 그 명령이 할 일은 Alias test를 뿌릴것이다.

alias로 humble만들기

    alias humble="source /opt/ros/humble/setup.bash; echo \"ROS2 humble is activated!\""
source ~/.bashrc도 alias 해두기

    alias sb="source ~/.bashrc; echo \"bashrc is reloaded\""
## 2-5 ROS_DOMAIN_ID 설정
ros2 도메인 설정

ros1과 ros2 차이점:DDS(Data Distribution Service)데이터 분산 서비스

DDS는 실시간 시스템의 실시간성, 규모가변성, 안전성, 고성능을 가능하게 하는 Object Management Group(OMG) 표준 출판/구속(Publish/Subscribe) 네트워크 커뮤니케이션 미들웨어이다.
domain id를 하나로 맞추면 같은 id의 device끼리 연결이된다.

    alias ros_domain="export ROS_DOMAIN_ID=13"
위의 alias humble에 ros_domain을 추가한다.

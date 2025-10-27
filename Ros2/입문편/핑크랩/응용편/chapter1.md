## 1.1 Package와 Node만들어보기
먼저 home에서 코드 작성을 위한 워크 스페이스를 만들자. 
```
mkdir -p ~/ros2_study/src
```
이 폴더에 이동한 후 colcon build를 수행한다.      
워크 스페이스는 소스 코드,빌드정보, 빌드할 때 로그정보가 저장되는 곳     
이제 첫 패키지를 만들어 보자. 
```
ros2 pkg create --build-type ament_python --node-name my_first_node my_first_package
```
이후 tree 명령어를 사용해보면 다음과 같이 나오는 것을 확인해볼 수 있다. tree: 폴더와 파일을 계층적으로 보여줌
```
.
└── my_first_package
    ├── my_first_package
    │   ├── __init__.py
    │   └── my_first_node.py
    ├── package.xml
    ├── resource
    │   └── my_first_package
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py

4 directories, 9 files

```
## 1.2 새로 만든 Node 실행해보기
src폴더에서 code.을 실행하자     
my_first_node.py의 내용이 저절로 작성되어있는 것을 확인해 볼 수 있다.       
<img width="929" height="333" alt="image" src="https://github.com/user-attachments/assets/25590fd4-6a21-4b04-8402-ec29943b3f34" />        

패키지를 만들기만 했기 때문에 다시 워크스페이스에 와 colcon build를 실행해준다. 
이제 노드를 실행해보려고 하면, 실행되지 않는다. 왜??--> 워크스페이스의 install 폴더를 확인해보면 local_setup.bash라는 얘가 있다. 우리는 얘를 source 명령으로 불러와야한다. 따라서 다음과 같이 작성해주고 노드를 실행해주면 된다. 
```
source ./install/local_setup.bash
```
```
ros2 run my_first_package my_first_node 
```
Hi from my_first_package.의 결과가 출력된다.      
source ./install/local_setup.bash를 쉽게 하기 위해 bashrc에 다음을 추가해주었다. 
```
alias ros2study='source ~/ros2_study/install/local_setup.bash'
```
## 1.5 내가 만든 package에서 topic 구독해보기
실행부분 코드는 이렇게 되어있다. 
```python
def main(args=None):
    rp.init(args=args)
    
    turtlesim_subscriber = TurtlesimSubscriber()
    rp.spin(turtlesim_subscriber)
    
    turtlesim_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
```
ros2 run 명령어로 이 노드를 실행시키면 바로 main문으로 진입한다. 먼저 init시키고, turtlesim_subscriber에 TurtlesimSubscriber함수 return값을 넣어준다. 이 class는 만들어 줄 것이다. 이것을 받아서 rp.spin(turtlesim_subscriber)로 계속 실행해 줄 것이다. node를 실행할때 터미널에서 ctrl+c를 눌러주면 나머지 코드가 실행될 것이다. 다시 우리는 위로 올라가서 TurtlesimSubscriber을 만들것이다. 
```python
class TurtlesimSubscriber(Node):
    
    def __init__(self):
        super().__init__('turtlesim_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback,
            10)
        self.subscription 
        
    def callback(self, msg):
        print("X : ", msg.x, ", Y : ", msg.y)
```
Node를 상속받아 TurtlesimSubscriber라는 이름의 class를 만들어준다. 먼저 __init__을 시작해주고 다음으로 super().__init__를 실행해 Node의 __init__을 실행해준다. 노드의 이름을 turtlesim_subscriber로 지정해준다. self.subscription라는 변수를 하나 만들어줘서, create_subscription해준다. 이 변수는 Pose라는 데이터 타입의 /turtle1/pose라는 토픽을 구독할것이고, 토픽이 들어올때마다 call_back함수를 실행할 것이다. 이 call_back함수는 토픽에서 구독한 msg를 print할 수 있게 해주었다.      
<img width="678" height="758" alt="image" src="https://github.com/user-attachments/assets/3c27b347-e7ad-4bdb-ac7e-0f3ce72710fd" />     
이렇게 만들고 setup.py에 가서 my_first_node를 찾은다음 콤마를 찍고 다음줄의 다음의 코드를 추가한다. 
```python
'my_subscriber = my_first_package.my_subscriber:main'
```
이 노드를 실행해본다면??       
<img width="681" height="514" alt="image" src="https://github.com/user-attachments/assets/bbbb1592-13ed-4b39-87c0-39f71eddec1a" />        

rqt_graph를 확인해본다면 다음과 같다.    
<img width="831" height="449" alt="image" src="https://github.com/user-attachments/assets/387c16df-9c63-40eb-b952-863d76f8551c" />     


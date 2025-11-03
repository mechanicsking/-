## 4.1 터미널에서 파라미터 다루기
<img width="1678" height="768" alt="image" src="https://github.com/user-attachments/assets/02eed377-74bc-45be-aad0-0369702b5346" />         

먼저 이렇게 준비해준다.  
이상태에서 다른 터미널에서 ros2 param list를 확인해보자.
```
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```
teleop_turtle에서 scale은 키보드를 눌렀을때 얼만큼 움직이는지의 크기를 나타내고 turtlesim에서 background는 배경색을 나타낸다는 것을 유추해볼 수 있다. 파라미터 값을 조회하고 싶다면??
```
ros2 param get /turtlesim background_g
```
```
Integer value is: 86
```
만약 파라미터의 값을 바꾸고 싶다면???
```
ros2 param set /turtlesim background_r 250
```
<img width="1514" height="763" alt="image" src="https://github.com/user-attachments/assets/af2bb2f4-819b-4ade-9fd6-87a96ab2d850" />      

색이 바뀐것을 확인해 볼 수 있다.      
이번에는 ros2_study에 my_first_package에 가서 params 폴더를 생성하자 그리고 그 폴더에서 param dump로 turtlesim params를 복사 붙여넣기 하자. 
```
ros2 param dump /turtlesim > ./turtlesim.yaml
```
dump된 param을 확인해 볼 수 있다.    
<img width="518" height="403" alt="image" src="https://github.com/user-attachments/assets/1f1e46fa-faab-43c1-aab5-0b47d4355c84" />       
parameter들을 일부 바꿔보고~/ros2_study/src/my_first_package/params 로 이동후 param load한다.    
```
ros2 param load /turtlesim ./turtlesim.yaml 
```
<img width="1407" height="767" alt="image" src="https://github.com/user-attachments/assets/bcf812c2-7c2e-4c44-a576-a6e27914fc32" />      

색이 바뀐걸 확인해 볼 수 있다. 
## 4.3 패키지 코드 내에서 파라미터 다루기
이번에는 코드로 파라미터를 다루자.    
DisTurtleServer코드에서 __init__에 다음의 두줄을 추가하자. 
```python
        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)
```
declare_parameter기능을 이용해서 quantile_time라는 이름을 0.75를 디폴트값으로 설정해주고, almost_goal_time라는 이름으로 0.95를 디폴트값으로 설정해준다.  빌드 후 액션 서버를 실행하고 param list를 확인해본다.       
<img width="1369" height="748" alt="image" src="https://github.com/user-attachments/assets/bd9e86bb-1794-4a78-9b59-e242cf93515a" />       

이제 param get 명령어로 이를 조회해보자.       
<img width="678" height="84" alt="image" src="https://github.com/user-attachments/assets/faad7d4b-d09b-4234-b514-c3ee0896f4ee" />       

이번에는 코드에서 파라미터를 가지고 올것이다.     
<img width="1012" height="448" alt="image" src="https://github.com/user-attachments/assets/17d28844-576c-413c-95aa-94a25af60203" />      

declare_parameter를 통해 우리는 선언했고, 밑에 get_parameters를 통해 누가 'quantile_time', 'almost_goal_time'를 선언하면 이것을 받아 각각의 변수로 받으라는 코드를 작성해주었다. 그리고 잘 받았는지 ㅣ확인하기 위해 print해준다.  declar_parameter만하면 코드내에서는 쓸수 없는 ros2의 parameter이고 내 코드내에서 쓰려면 변수로 설정되어야하기에 위와 같이 적어주었다.  이제 colcon build해준다음에 action을 실행해주고 파라미터를 변경해보자.       
<img width="1359" height="765" alt="image" src="https://github.com/user-attachments/assets/92282050-34db-4f7d-b477-725a9003bc0c" />       

잘 바뀌는 것을 확인해 볼 수 있다.       
만약 파라미터의 변경을 실시간으로 알고 싶다면 먼저 다음을 import해준다. 
```python
from rcl_interfaces.msg import SetParametersResult
```
이렇게 import해준다음 __init__에 파라미터 콜백 함수를 작성해준다. 
```python
        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)
        
        (quantile_time, almosts_time) = self.get_parameters(
            ['quantile_time', 'almost_goal_time'])
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def parameter_callback(self, params):
        for param in params:
            print(param.name, "is changed to ", param.value)
            
        return SetParametersResult(successful=True)
```
여기서 add_on_set_parameters_callback은 누가 파라미터를 바꾸면 바로 반응하라는 명령어이다. 따라서 parametr_callback이라는 함수를 만들었고 이를 통해 반응해주었다. parametr_callback에서는 바뀐 파라미터 정보를 가지고 와서 param.name과 param.value를 출력해준다. 이제 파라미터를 변경하면 터미널에 바로 나올 것이다. 빌드 후 액션 서버를 실행해보자.       
<img width="1359" height="765" alt="image" src="https://github.com/user-attachments/assets/66d90567-0d74-4126-82fd-0161b926cee2" />       

이번에는 실제 코드 내부 변수에 적용해보자.      
```python
        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)
        
        (quantile_time, almosts_time) = self.get_parameters(
            ['quantile_time', 'almost_goal_time'])
        self.quantile_time = quantile_time.value
        self.almosts_time = almosts_time.value
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def parameter_callback(self, params):
        for param in params:
            print(param.name, "is changed to ", param.value)
            
            if param.name == 'quantile_time':
                self.quantile.time = param.value
            if param.name == 'almost_goal_time':
                self.almosts_time = param.value
                
        print('quantile_time and almost_goal_time is ',
              self.quantile_time, self.almost_time)
            
        return SetParametersResult(successful=True)
```
가지고 온 변수의 내용들을 self로 잡아주었다. 이후 callback함수에서는 바뀐게 있으면 self의 값도 변경할수있게 만들어주었다. 그리고 변경된 내용도 프린트 한다.       
<img width="1359" height="765" alt="image" src="https://github.com/user-attachments/assets/3944bb98-13f0-4a07-b35e-70dd7835a8b6" />         


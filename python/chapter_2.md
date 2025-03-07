visual studio code에서 파일을 저장할때는 ctrl+s
## 2-1 숫자 자료형
자료형-->자료의 형태

    print(숫자)
    print(간단한 연산)
## 2-2 문자열 자료형
    print('풍선')
    print("나비")
    print("ㅋ"*9) #문자형과 숫자형을 섞어서 표현할수 있다.
## 2-3 boolean 자료형
참과 거짓을 의미

    print(5>10)
terminal에 false라고 생성

    print(5<10)
terminal에 true라고 생성

    print(True)
    print(False)
    print(not True) #거짓
    print(not (5>10)) #true
## 2-4 변수
    animal = "강아지"
    name = "연탄이"
    age = 4
    hobby ="산책"
    is_adualt = age>=3
    print("우리집 "+animal+"의 이름은 "+name+"이에요")
    print(name +"은(는)"+ str(age) +"살이며, " + hobby +"을 아주 좋아해요")
    print(name +"는 어른일까요? " + str(is_adult))

문자형 변수를 넣는 방법은 + 변수 +

정수형과 boolean형 변수를 문자형으로 바꾸는 방법은 + str(변수) +
    
,로 분리 할 수 있다. 이때는 정수형 변수와 booleam형 변수를 문자형으로 바꾸지 않아도 된다. but, ,는 띄어쓰기가 포함된다.

    print(name, "은(는)", age, "살이며, ", hobby, "을 아주 좋아해요")
## 2-5 주석
    #으로 주석처리함

    '''여러문장 주석처리'''
여러문장을 일괄적으로 주석처리하고 싶으면 드래그해서 ctrl+/

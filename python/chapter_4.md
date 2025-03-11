## 4-1 문자열
    sentence = '나는 소년입니다'
    print(sentence)
    sentence2 = "파이썬은 쉬워요"
    print(sentence2)
    sentence3 = """
    나는 소년이고,
    파이썬은 쉬워요
    """
    print(sentence3)
## 4-2 슬라이싱
    jumin = "123456-7891011" #이 정보중에서 필요한 부분만 떼어서 사용하는 것을 슬라이싱이라고 한다.
    print("성별 : " + jumin[7]) #변수[몇 번째]
    print("연 : " + jumin[0:2]) #변수[좌변 포함, 우변 미만]
    print("월 : " + jumin[2:4])
    print("생년월일 : " + jumin[:6]) #처음부터 6직전까지
    print("뒤 4자리 : " + jumin[-4:]) 
## 4-3 문자열처리함수
    python ="Python is Amazing"
    print(python.lower())  #문자열.lower -->문자열이 소문자로 출력
    print(python.upper())  #문자열.upper -->문자열이 대문자로 출력
    print(python[0].isupper()) #문자열[몇번째].isupper-->몇번째 알파벳이 대문자인가?-->True
    print(len(python)) #len(문자열)-->문자열 길이를 반환해줌-->17
    print(python.replace("Python", "Java")) #바꾸고 싶은 문자가 있을때-->문자열.replace("바꾸고 싶은 문자", "바꿀 문자")

    index = python.index("n") #어떤 문자가 어느 위치에 있는지 확인하고 싶을 때-->문자열.index("찾고싶은 문자")
    print(index)
    index = python.index("n", index+1) #문자열.index("찾고싶은 문자",스타트 위치)
    print(index)

    print(python.find("Java")) #원하는 값이 없을 때 -1값을 출력& 프로그램 종료하지 않음
    print(python.index("Java")) #원하는 값이 없을 때 오류를 내면서 프로그램 종료
    print(python.count("n")) #count-->단어가 몇번 나오는지 값을 출력해줌-->2
## 4-4 문자열포맷
    print("a" +"b")
    print("a", "b")

    #방법1
    print("나는 %d살입니다." %20) #%d에서 d는 정수값을 의미하기에 정수값만 넣을수있다.
    print("나는 %s을 좋아해요." %"파이썬") #%s에서 s는 문자열을 넣겠다를 의미
    print("Apple 은 %c로 시작해요." %"A") #%c에서 c는 캐릭터를 의미 한글자만 받는다
    print("나는 %s색과 %s색을 좋아해요." %("파란", "빨간"))

    #방법2
    print("나는 {}살입니다." .format(20)) #format속에 있는 값을 {}사이에 넣어준다.
    print("나는 {0}색과 {1}색을 좋아해요." .format("파란", "빨간")) #{0}에는 0번째에 있는 파란이 들어가고 {1}에는 1번째에 있는 빨간이 들어간다.

    #방법3
    print("나는 {age}살이며, {color}색을 좋아해요." .format(age =20,color="빨간"))

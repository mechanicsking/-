## 3-1 연산자
    print(1+1) #2
    print(3-2) #1
    print(5*2) #10
    print(6/3) #2
    print(2**3) #2^3=8
    print(5%3) #나머지 구하기 2
    print(5//3) #몫 구하기 1

    print(10>3) #True
    print(4>=7) #False

    print(3 == 3) #좌변과 우변이 똑같은지 확인-->True
    print(3 + 4 == 7) #True
    print(3 != 1) #좌변과 우변이 같지 않다-->True
    print(not(3 != 1)) #False
    
    print((3>0) and (3<5)) #앞에 항과 뒤에 항이 모두 만족해야 True를 출력함-->True
    print((3>0) & (3<5)) #True

    print((3>0) or (3>5)) #앞에 항이나 뒤에 항중 하나라도 true라면 true출력-->True
    print((3>0) | (3>5)) #True
## 3-2 간단한 수식
    print((2+3)*4) #20
    number = 5
    number = number+3 #5+3=8
    number += 3 #8+3=11
    print(number)
    number *= 2 #11*2=22
    print(number)
## 3-3 숫자처리함수
    print(abs(-5)) #abs는 절대값이다-->5
    print(pow(4,2)) #pow는 제고# 3-1 연산자
    print(1+1) #2
    print(3-2) #1
    print(5*2) #10
    print(6/3) #2
    print(2**3) #2^3=8
    print(5%3) #나머지 구하기 2
    print(5//3) #몫 구하기 1

    print(10>3) #True
    print(4>=7) #False

    print(3 == 3) #좌변과 우변이 똑같은지 확인-->True
    print(3 + 4 == 7) #True
    print(3 != 1) #좌변과 우변이 같지 않다-->True
    print(not(3 != 1)) #False
    
    print((3>0) and (3<5)) #앞에 항과 뒤에 항이 모두 만족해야 True를 출력함-->True
    print((3>0) & (3<5)) #True

    print((3>0) or (3>5)) #앞에 항이나 뒤에 항중 하나라도 true라면 true출력-->True
    print((3>0) | (3>5)) #True
## 3-2 간단한 수식
    print((2+3)*4) #20
    number = 5
    number = number+3 #5+3=8
    number += 3 #8+3=11
    print(number)
    number *= 2 #11*2=22
    print(number)
## 3-3 숫자처리함수
    print(abs(-5)) #abs는 절대값이다-->5
    print(pow(4,2)) #pow는 거듭제곱이다.-->4^2=16
    print(max(5,12)) #max는 최댓값을 찾아서 반환해줌-->12
    print(min(5,12)) #min은 최솟값을 찾아서 반환해줌-->5
    print(round(3.14)) #round는 반올림을 해줌-->3
    
    from math import * #math 라이브러리 안에 있는 모든 것을 이용하겟다
    print(floor(4.99)) #floor은 내림이다-->4
    print(ceil(3.14)) #ceil은 올림이다-->4
    print(sqrt(16)) #sqrt는 제곱근-->4
    

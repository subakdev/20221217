# [turtlesim과 rqt 사용하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. turtlesim 설치하기
   2. turtlesim 시작하기
   3. turtlesim 사용하기
   4. rqt 설치하기
   5. rqt 사용하기
   6. Remapping
   7. turtlesim 닫기 
5. 요약

## 목표
* turtlesim pakcage 설치 및 사용법 익히기
* rqt 도구 설치 및 사용법 익히기
## 배경지식
* Turtlesim은 ROS2를 배워보기 위해서 제공하는 간단한 시뮬레이터
* ROS2를 할 수 있는 간단한 예제들을 경험
* 이를 바탕으로 실제 robot으로 무엇을 할 수 있을지 감잡기
* rqt
  * GUI 도구
  * rqt에서 할 수 있는 모든 것을 command line에서도 가능
  * 하지만 UI로 하면 좀더 쉽게 시작할 수 있는 장점
* 이후 ROS2 핵심 개념을 소개
  * node
  * topic
  * service
* 여기서는 설정하고 실행해보는 것에 초점 
## 사전준비
* 환경설정 완료하기

## 실습
### 1. turtlesim 설치하기
* 먼저 터미널을 열어서 setup 파일에 대해서 source 명령 실행
* ROS2 배포판(distro)에 맞는 turtlesim package 설치하기
```bash
sudo apt update

sudo apt install ros-humble-turtlesim
```

* 설치가 제대로 되었는지 확인하기 (turtlesim package의 실행자 확인)
```
ros2 pkg executables turtlesim
```

* 정상적으로 설치된 경우 출력 메시지(turtlesim의 실행자 목록을 출력) :
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### 2. turtlesim 구동시키기
* turtlesim을 구동시키기 위해서 아래 명령을 터미널에서 수행
```bash
ros2 run turtlesim turtlesim_node
```

* 시뮬레이터 윈도우가 화면에 나타나고 화면 중앙에 turtle이 보인다.

![](https://docs.ros.org/en/humble/_images/turtlesim.png)

* 명령을 내린 터미널에서 다음과 같은 메시지가 나타난다.
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim

[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

* 기본 turtle 이름은 turtle1이고 기본 좌표를 생성한다.

### 3. turtlesim 사용하기
* 새로운 터미널을 열고 source 명령 실행하기
* 첫번째 node에서 turtle을 제어하기 위해서 새로운 node를 실행한다.
```
ros2 run turtlesim turtle_teleop_key
```

* 현재까지 3개 터미널 윈도우를 열었다. 
  1. turtlesim_node 을 실행하는 터미널
  2. turtle_teleop_key 을 실행하는 터미널
  3. turtlesim window 을 실행하는 터미널

주의) 화살표 키를 이용해서 거북이를 이동시키면 일정 거리 이동 후 정지한다. 현실에서도 로봇을 사람이 조종할 때와 유사하게 사용자가 조종하지 않을 때 로봇과의 연결이 끊기는 상황과 동일하도록 구현 함.

* list 명령을 사용하여 노드와 관련된 service, topic, action을 확인 할 수 있다. 
```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### 4. rqt 설치하기 
* rqt 설치
```
sudo apt update
sudo apt install ~nros-humble-rqt*
```

* rqt 실행
```
rqt
```

### 5. rqt 사용하기

* 서비스 rqt로 확인하기
상단 메뉴바 : Plugins > Services > Service Caller

주의) rqt가 모든 플러그인을 찾는데 시간이 걸릴 수 있다. 플러그인을 클릭했지만 옵션 선택이 안되는 경우 rqt를 닫고 rqt --force-discover 명령을 입력 해야 한다.

새로고침 버튼을 사용하여 turtle 노드의 모든 서비스를 사용 할 수 있는지 확인한다.
/spawn 서비스를 선택한다.


### 5.1 spawn 서비스 사용하기 

rqt를 사용하여 /spawn를 선택하고 "call"을 클릭하면 새로운 거북이가 생성된다.(Response tap에서 확인가능)

Response tap의 'Value' 항복을 두 번 클릭하여 이름을 지정 할 수 있다.

Request tap에서 새로 생성될 거북의의 위치(좌표)를 입력 할 수 있다.(x=1.0, y=1.0)

![](https://docs.ros.org/en/humble/_images/spawn.png)

주의) 기존에 생성되어있는 거북이 이름과 동일하게 생성하려고 하면 turtsim_node 실행하는 터미널에서 오류 메시지가 표시된다.

### 5.2 set_pen 서비스 사용하기 

set_pen 서비스 : 거북이가 지나간 선을 표시 해주는 서비스 

![](https://docs.ros.org/en/humble/_images/set_pen.png)

Request Tap의 r,g,b 토픽의 값을 0-255사이의 값으로 설정을 하고, width 선의 두께를 설정한다. 

빨간색 선은 r값을 255, width값을 5로 변경한다. 그리고 "call"버튼을 눌러 반영한다.

turtle_teleop_key가 실행되고 있는 터미널에서 화살표 키를 누르면 turtle1 펜이 변경된 것을 볼 수 있다.

![](https://docs.ros.org/en/humble/_images/new_pen.png)

turtle2는 움직일 방법이 없다는 것을 확인 할 수 있다. turtle1의 cmd_vel 항목을 turtle2에 다시 매핑하여 이 작업을 수행 할 수 있다. 

### 6. 리맵핑
새 터미널에서 source 적용 후 다음 코드를 실행한다 : 

```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
이제 새로운 터미널에서 turtle2를 이동 시킬 수 있고, 기존의 터미널에서 turtle1을 이돌 시킬수 있다.

![](https://docs.ros.org/en/humble/_images/remap.png)

### 7. turtlesim 종료하기
* 시뮬레이션 종류
  * turtlesim_node 터미널에서 Ctrl+C을 눌러 종료
  * teleop(키로 조정하는) 터미널에서는 q를 눌러 종료한다

## 요약
* turtursim과 rqt를 사용하여 ROS2의 핵심 개념을 익혀보았다

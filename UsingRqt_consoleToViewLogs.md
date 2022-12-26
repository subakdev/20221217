# [rqt_console을 사용하여 logs 보기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html#)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. rqt_console에서 메시지
   3. Logger level
5. 요약

## 목표
* log 메시지를 조사하는데 사용하는 rqt_console에 대해서 알아본다.
## 배경지식
* rqt_console은
  * GUI 도구
  * ROS2에서 log 메시지를 조사하는 도구
* 일반적으로 log 메시지는 터미널에 표시된다. 
* rqt_console로 시간에 따라 이 메시지들을 수집하고 체계적인 방식으로 내용을 볼 수 있고 저장된 파일을 조사하기 위해서 리로드가 가능하다.
* nodes는 여러 가지 방식으로 관심있는 event와 status의 메시지를 출력하는데 logs를 사용한다. 내용은 사용자를 위한 도움되는 정보이다.
## 사전준비
* rqt_console와 tuirtlesim 설치
* 새 터미널을 열때마다 ROS 환경에 대해서 source를 수행한다.
## 실습
###   1. 설정
* 새 터미널을 열고 rqt_console을 구동시키기 위해 아래 명령을 수행한다.
```
ros2 run rqt_console rqt_console
```
* rqt_console 창이 열린다.

![](https://docs.ros.org/en/humble/_images/console.png)

* console의 첫번째 섹션은 system의 log 메시지를 보여준다.
* 가운데 창인 "Exclude Messages"에서 배제할 필터링 옵션을 추가할 수 있다. 추가할려면 오른쪽에 "+" 버튼을 눌려서 필터를 추가시킬 수 있다.

* 맨맡에 창은 우리가 입력한 string을 포함하는 메시지를 하이라이트해준다. 다양한 string을 추가시킬 수 있다.

* 이제 새 터미널에서 turtlesim을 아래 명령으로 실행하자
```
ros2 run turtlesim turtlesim_node
```

###   2. rqt_console에서 메시지
* rqt_console이 log 메시지를 출력할려면, turtle을 벽쪽 충돌시키자. 
* 새 터미널에서 아래와 같이 ros2 topic pub 명령을 입력한다.
```
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```

* 위에 명령은 일정한 rate로 topic을 publish한다. 
* 따라서 turtle이 계속 벽을 들이박는다. 
* rqt_console 내에서 Warn 레벨의 동일한 메시지를 계속해서 출력하게 된다. (아래와 같이)

![](https://docs.ros.org/en/humble/_images/warn.png)

* ros2 topic pub 명령을 실행했던 터미널 내에서 Ctrl+C 를 눌러서 종료시키면 더이상 turtle이 벽에 부딪히지 않는다.

###   3. Logger level
* ROS2의 logger level의 중요도 순
```
Fatal
Error
Warn
Info
Debug
```
* 사실 각 level이 표시하는 정확한 표준이 있는 것은 아니다. 하지만 다음과 같은 가정을 하고 사용한다.
  * Fatal
    * 시스템을 종료해야할만큼 중요한 문제
  * Error
    * 정상적인 동작이 문제가 생길 수 있는 문제
  * Warn
    * 기대하지 못한 활동이나 비이상적인 결과가 생길 수 있는 문제
  * Info
    * event와 status 표시
  * Debug
    * 시스템 실행의 상세 단계를 보기 위해서
* default level은 Info
* 따라서 Info 이상의 중요도를 가지는 level을 표시하게 된다.
* 일반적으로 Debug 메시지는 hidden으로 설정. Info보다 중요도가 낮으므로
* ex) default level을 Warn으로 설정하면, Warn, Error, Fatal 메시지를 볼 수 있다.

### 3.1 default logger level 설정하기
* 처음 /turtlesim node를 실행할때 default logger level을 설정할 수 있다.
* 다음 명령을 실행해 보자.
```
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```
* 이제 default level을 WARN으로 설정하였으니 WARN보다 아래에 있는 Info level 메시지는 표시되지 않는다.

## 요약
* system의 log 메시지를 면밀히 조사하는데 rqt_console가 매우 도움이 된다. 
* 다양한 원인으로 log 메시지를 조사하기를 원할 수 있다.
  * 어느 부분에서 문제가 발생했는지
  * 어떤 events들이 발생해서 문제가 생겼는지
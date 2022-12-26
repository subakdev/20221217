# [topics 이해](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. rqt_graph
   3. ros2 topic list
   4. ros2 topic echo
   5. ros2 topic info
   6. ros2 interface show
   7. ros2 topic pub
   8. ros2 topic hz
   9. clean up
5. 요약

## 목표
* rqt_graph 사용하기
* command line 도구 사용하기
* ROS2 topics 살펴보기
## 배경지식
* ROS2는 복잡한 시스템을 여러개의 모듈의 nodes로 분해
* topics은 ROS2 graph의 핵심 요소
  * nodes이 서로 message를 교환하기 위한 bus처럼 동작
![](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)

* node는 data를 여러 topics을 통해서 publish
* 동시에 여러 topics를 통해서 subscribe

![](https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

* topics
  * nodes 사이에 data를 이동시키는 핵심 방법 중에 하나
## 사전준비
* 이전 튜터리얼에서 nodes에 대한 기본 지식을 제공
* 새 터미널을 열면 항상 source 명령 실행
## 실습
### 1. 설정
* 이제 turtlesim을 구동시키는 것이 편해졌죠?
* 새로운 터미널 열어서 다음 명령 실행
```
ros2 run turtlesim turtlesim_node
```
* 또 새로운 터미널 열어서 다음 명령 실행
```
ros2 run turtlesim turtle_teleop_key
```
* 이전 튜터리얼 회고
  * 이 nodes의 이름
    * /turtlesim
    * /teleop_turtle

### 2. rqt_graph
* 전체 튜터리얼에서 rqt_grph를 사용하여 확인할 수 있는 것
  1. nodes 사이의 연결
  2. nodes와 topics 변화를 시각화
* rqt_graph 실행하기 위해서 새로운 터미널 열고 다음 명령을 실행
```bash
rqt_graph
```

* rqt를 열어서 Plugins > Introspection > Node Graph 선택하기 (rqt에서 rqt_graph 동일 화면 확인 가능)
![](https://docs.ros.org/en/humble/_images/rqt_graph.png)

* 위 그림에서 graph 주변으로 2개 actions과 nodes와 topics를 볼 수 있다. 만약에 중앙에 있는 topic 위에 마우스를 가져가면 위에 있는 그림처럼 색상이 하이라이트된다. 
* graph는 /turtlesim node와 /teleop_turtle node가 하나의 topic으로 서로 통신하는 것을 보여준다. 
* /teleop_turtle node는 /turtle1/cmd_vel topic으로 data를 publish하고 /turtlesim node는 data를 수신하기 위해서 해당 topic으로 subscribe한다.
* rqt_graph의 하이라이트 기능은 특히 nodes가 많고 topics이 복잡하게 연결되어 있는 경우 도움이 많이 된다.
* rqt_graph는 시각적인 내부를 살펴보는 도구로서 이제 topics을 살펴보기 위한 command line 도구이다.

### 3. ros2 topic list
* ros2 topic list 명령을 새로운 터미널에서 실행해보자.
* 현재 시스템에서 활성화되어 있는 모든 topcis 목록을 보여준다.
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
* ros2 topic list -t 를 실행해보자.
* 이번에도 topics 목록을 보여주는데 topic 타입을 [] 안에 표현해준다.
```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```
* rqt_graph에서 이런 모든 topics이 어디에서 볼 수 있을까? Hide 아래에 있는 박스를 uncheck해주면 된다.
![](https://docs.ros.org/en/humble/_images/unhide.png)
* 이 옵션을 check된 상태로 남겨두자. 
  
### 4. ros2 topic echo
* topic에서 publish되는 data를 보기 위해서 아래 명령을 사용한다.
```
ros2 topic echo <topic_name>
```
* /teleop_turtle이 /turtlesim 데이터를 /turtle1/cmd_vel 토픽을 통해 publish한다는 것을 알았음으로 echo명령을 통해 토픽을 확인해 보자:
```
ros2 topic echo /turtle1/cmd_vel
```
* 처음에는 아무런 데이터도 반환하지 않는다.
  * /teleop_turtle 이 publish할때 까지 기다리기 때문이다.
* turtle_teleop_key가 실행 중인 터미널로 돌아와서 키보드의 화실키로 turtle을 움직여보자.
* echo를 실행 중인 터미널을 보면, 매번 움직임에 대한 position data를 볼 수 있다.

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```
* rqt_graph로 와서 Debug 박스를 체크를 해제한다.

![](https://docs.ros.org/en/humble/_images/debug.png)

* /_ros2cli_26646 는 무엇일까?
  * node
  * echo 명령으로 생성됨
* 이제 cmd_vel topic으로 publisher가 publish하는 data를 볼 수 있고 2개 subscribers가 수신한다.

### 5. ros2 topic info

* topic은 point-to-point(1대1) 통신뿐만 아니라 일대다, 다대다 통신도 가능하다. 

* 아래 명령을 수행하여 확인할 수 있다 : 
```
ros2 topic info /turtle1/cmd_vel
```

* 실행한 결과는 다음과 같다 : 
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```
* Publisher node(teleop_turtle)가 1개, Subscriber가 2개 (ros2cli, turtlesim)

### 6. ros2 interface show

* nodes는 topics으로 messages를 보낸다.
* publishers와 subscribers는 반드시 동일한 타입의 메시지를 주고 받아야만 한다. -> publishers와 subscribers가 주고 받는 메시지의 타입은 동일해야만 한다.

* 앞서 ros2 topic list -t를 실행시킨 뒤 본 토픽 타입은 각 토픽이 어떤 메시지 타입을 사용했는지 볼수있었다. 
cmd_vel 토픽 타입을 아래와 같은 타입이라는 것을 기억하자. : 

```
geometry_msgs/msg/Twist
```
* geometry_msgs 패키지내의 Twist라는 msg가 있다는 것을 뜻한다. 


* 이제 ros2 interface show <msg type>을 실행하여 메시지에 대해 자세히 알 수 있다.
해당 message가 가지는 데이터 구조가 무엇인지 볼수 있다

```
ros2 interface show geometry_msgs/msg/Twist
```
호출 결과 : 
```
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```
/turtlesim node는 2개 vectors(linear, angular)를 가지고 있고 각각은 3개 elements를 갖는다. /teleop_turtle가 /turtlesim에게 전달하는 data를 echo 명령을 실행해서 봤던 data가 기억난다면 동일한 구조를 갖고 있다. : 
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

### 7. ros2 topic pub 

* 이제 메시지 구조를 알고 있으므로 command line에서 직접 topic으로 data를 publish 할 수 있다:
```
ros2 topic pub <topic_name> <msg_type> '<args>'
```

'<args>' 인자는 실제 topic으로 전달할 data이다. 바로 위에서 확인한 구조와 같이 

이 인자는 YAML syntax으로 입력해야 해야하기 때문에 주의가 필요하다
다음과 같이 전체 명령을 입력한다 : 
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

* '--once'는 옵션 인자로 "한번 publish한 뒤 종료"를 의미한다.

* 터미널에서 다음과 같은 메시지를 받을 것이다:
```
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```

그리고 turtle이 아래와 같이 움직일 것이다 :

![](https://docs.ros.org/en/humble/_images/pub_once.png)

이 turtle(일반적으로 emulation하려는 실제 로봇)은 지속적 동작을 위한 commands를 필요로한다. 
따라서 turtle을 계속 움직이게 하기 위해서는 다음 코드를 실행한다 : 
```
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
--once 옵션과 --rate 1 옵션의 차이는 1Hz로 주기적인 스트림으로 ros2 topic pub command를 publish 한다. 
![](https://docs.ros.org/en/humble/_images/pub_stream.png)

rqt_graph를 refresh 해보면 무슨 상황인지 시각적으로 확인 해 볼수 있다.
ros2 topic pub..노드 (/_ros2cli_30358)은 /turtle1/cmd_vel 토픽을 publish하고 있다.
ros2 topic echo ... 노드(/_ros2cli_26646)과 /turtlesim 노드 두개의 노드 모두 해당 메시지를 받고 있다. 
![](https://docs.ros.org/en/humble/_images/rqt_graph2.png)
마지막으로, pose 토픽을 echo로 실행하고 rqt_graph로 확인 할 수 있다:

```
ros2 topic echo /turtle1/pose
```
![](https://docs.ros.org/en/humble/_images/rqt_graph3.png)

/turtlesim노드가 pose 토픽을 publish하고 있고 echo 노드가 subscribe 하고 있는 것을 확인 할 수 있다.

### 8. ros2 topic hz

이 프로세스에 대한 마지막 소개는, publish 되는 데이터의 rate를 다음 명령으로 확인 할 수 있다:
```
ros2 topic hz /turtle1/pose
```

/turtlesim 노드가 pose 토픽으로 publsih 한 데이터의 rate 값을 반환한다
```
average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
```

turtle1/cmd_vel을 1hz로 일정하게 publish 했던 ros2 topic pub --rate 1 명령을 다시 호출 한다. 위의 command를 turtle1/pose 대신 turtle1/cmd_vel을 입력하면 average rate를 확인 할 수 있을 것이다. 



### 9. node 동작 중지
* 각 터미널에서 Ctrl+C 키를 입력해보자.
* 해당 터미널에 있는 node 실행이 중지된다.
## 요약
* nodes는 topic으로 정보를 publish한다. 다른 nodes은 해당 정보를 접근하기 위해서 subscribe할 수 있다. 
이 튜토리얼에서 여러 nodes이 topics으로 연결되어 있다는 것을 rqt_graph와 command line 도구로 알아보았다. 이제 ROS2 시스템에서 data가 이동하는 개념을 이해할 수 있었다.

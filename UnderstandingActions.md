# [actions 이해하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. actions 사용하기
   3. ros2 node info
   4. ros2 action list
   5. ros2 action info
   6. ros2 interface show
   7. ros2 action send_goal
5. 요약

## 목표
* ROS2 action 살펴보기 
## 배경지식
* Action은 ROS2에서 제공하는 통신 타입 중에 하나이다. 
  * 특히 오랜 시간 동안 실행(long running)되는 task에 적합하다. 

* actions은 우리가 배운 topics과 services를 이용하여 구현되었다. (즉 topic과 services 두가지 특징을 다 가진다는 뜻)
* service와 비교
  * cancel 기능
    * actions의 기능은 services와 유사한데 actions에는 cancel 기능을 제공한다는 것이 차이점이다.
  * feedback
    * 추가로 actions은 지속적인 feedback(steady feedback)이 제공되는데 이것은 services가 한번의 response를 반환하는 점과 반대되는 점이다.
* topics과 비교
  * goal 
    * actions은 client-server 모델을 사용하며 이는 publisher-subscriber 모델과 유사하다.
    * “action client” node는 goal을 “action server” node에게 전달한다. “action server” node는 goal에 대한 ack를 보내고 feedback stream과 result를 반환한다.

![](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)

## 사전준비
* 이전에 배운 nodes와 topics 개념을 이용
* turtlesim package를 사용하여 진행한다.
* 항상 새 터미널을 열면 ROS2 환경을 source 하도록 하자.

## 실습
### 1. 설정 
* 2개 nodes인 /turtlesim, /teleop_turtle 를 실행한다 

* 새로운 터미널에서 실행 : 
```
ros2 run turtlesim turtlesim_node
```

* 다른 새로운 터미널에서 실행 : 
```
ros2 run turtlesim turtle_teleop_key
```
### 2. actions 사용하기

* /teleop_turtle node를 launch할때, 다음과 같은 메시지가 출력된다 :
```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

* 2번째 줄이 action과 관련된 부분이다. (첫번째 줄은 “cmd_vel” topic과 관련된 topic 튜터리얼에서 이미 다뤘다.)
* G|B|V|C|D|E|R|T 는 QWERTY 키보드 자판에서 F키 주변에 박스 형태로 위치하고 있는 키들이다.
* F 주변에 있는 각 키의 위치 turtlesim의 방향에 대응된다. 예제로 E는 좌상 코너 방향까지 turtle을 회전시킨다.

* /turtlesim node가 실행 중인 터미널 창을 보자.
* key를 누를때마다 action server(/turtlesim node의 일부)에 goal을 전송한다.
* 이 goal은 turtle을 특정 방향까지 회전시킨다. turtle이 회전을 완료하게 되면 goal의 결과를 화면에 표시한다.

```
[INFO] [turtlesim]: Rotation goal completed successfully
```

* F키는 실행 중간에 goal 달성을 취소(cancel)시킨다.

* C키를 누르고 F키를 누르면 아래와 같은 메시지가 출력된다. (회전 중일때 F키를 눌러야 cancel이 됨)
* 실행중인 터미널에 다음과 같은 메시지가 표시된다 : 
```
[INFO] [turtlesim]: Rotation goal canceled
```

* 클라이언트 쪽에서 뿐만 아니라 서버쪽에서도 goal을 정지 시킬 수 있다. 서버쪽에서 진행중인 goal에 대한 정지를 선택하면 goal을 "abort"한다. 

* D키를 누르고, 처음 회전이 완료되기 전에 G 키를 눌러보자. /turtlesim 노드가 실행되는 터미널에서 다음과 같은 메시지가 출력된다. :
```
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

* action 서버가 현재 goal을 abort시키는 경우는 새로운 goal이 들어온 경우이다. 
* 새로 들어온 goal을 선택하는 것 이외에 아래와 같은 선택 가능
  * 새로 들어온 goal을 reject
  * 현재 goal을 완료하고 나서 새로 들어온 goal을 실행
* 즉 모든 action server가 새로운 goal이 들어오면 현재 goal을 abort 시킨다고 가정하면 안됨!!!

### 3. ros2 node info

* /tuetlesim node의 action을 보기 위해서 새로운 터미널을 열고 아래 명령을 실행한다. : 
```
ros2 node info /turtlesim
```
* 명령의 결과로 /turtlesim의 subscribers, publishers, services, action servers, action clients의 목록을 확인 할 수 있다 : 

```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

* /turtlesim의 /turtle1/rotate_absolute action은 action server내에 있다.
즉, /turtlesim은 /turtle1/roate_absolute action에 대해 응답하고 feedback을 제공한다.

* /teleop_turtle node는 Action clients내에 /turtle1/rotate_absoulte 이름을 갖는다. 즉 Action Clients가 해당 action 이름으로 goal을 보낸다는 뜻이다.

```
ros2 node info /teleop_turtle
```

* 실행결과 :
```
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

### 4. ros2 action list 
* ROS graph내 있는 모든 actions을 보려면 아래 명령을 실행한다 :
```
ros2 action list
```

* 결과로 다음을 출력한다. : 
```
/turtle1/rotate_absolute
```

* 현재 ROS graph 내에는 action이 1개만 있다. 이전에 본바와 같이 이것이 turtle의 rotation을 제어한다.
* 이미 알고 있듯이 하나의 action client(/teleop_turtle에 존재)와 하나의 action server(/turtlesim)이 있다.
* action을 위해서 ros2 node info <node_name> 명령을 사용하여 확인 가능!

### 4.1 ros2 action list -t
* action도 types을 가지고 있다. (topics, services도 types이 있었다.) /turtle1/rotate_absolute의 type을 찾으려면 아래 명령을 수행한다. : 
```
ros2 action list -t
```
* 다음을 출력한다 : 
```
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```
* action 이름(/turtle1/rotate_absolute) 오른쪽에 있는 괄호안에 있는 turtlesim/action/RotateAbsolute이 바로 action type이 있다.
* command line에서 action을 실행하고자 하면 이것이 필요하다. 

### 5 ros2 action info 
* 아래 명령으로 /turtle1/rotate_absolute action 내부를 볼 수 있다:

```
ros2 action info /turtle1/rotate_absolute
```
* 다음을 출력한다 : 
```
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
* 출력으로부터 알 수 있는 것
  */turtle1/rotate_absolute action을 대해서
    * /teleop_turtle node는 action client를 가진다
    * /turtlesim node는 action server를 가진다. 

### 6 ros2 interface show
* action goal을 자체적으로 보내거나 실행하기 전에 한가지 더 필요한 정보가 바로 action type의 structure이다.

* ros2 action list -t 명령을 실행하여 /turtle1/rotate_absolute의 type을 확인했던 것을 기억할 것이다.
* 터미널에서 action type으로 명령을 실행한다.
```
ros2 interface show turtlesim/action/RotateAbsolute
```
* 결과를 출력한다. : 
```
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
* --- 으로 나눠진 구조 
  * 첫번째 부분 : goal에 대한 request structure
  * 두번째 부분 : goal에 대한 result structure
  * 세번째 부분 : goal에 대한 feedback structure

### 7 ros2 action send_goal
* 자 이제 터미널로 action goal을 전송해보자.
```
ros2 action send_goal <action_name> <action_type> <values>
```
*  <values>에는 YAML 포맷이 들어간다.

* turtlesim window를 띄워놓고 아래 명령을 터미널에서 실행한다. :
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```
* turtle이 회전하는 것을 볼 수 있고 터미널에서는 아래와 같은 메시지를 볼 수 있다.:
```
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```
* 모든 goals은 ID를 가지고 있다. (위에 return 메시지에서 볼수 있듯이) 결과를 보면 delta 라는 필드가 있는데 이것은 시작 지점으로부터의 이동 거리이다. 

* 이 goal에 대한 feedback을 보기 위해서 ros2 action send_goal 명령에 --feedback을 추가한다. :
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

* 출력 결과 : 
```
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```

* goal이 완료될때까지 남은 각도(radians)를 feedback으로 계속 수신하게 된다.

## 요약
* actions는 services와 유사하다.
  * 오래 실행되는(long running) task에 적합
  * 주기적인 feedback 제공
  * 작업 중간에 cancel 기능 제공
* robot 시스템에서 navigation에 action 사용이 적합
  * robot에게 action goal 전달(goal : 이동해야 하는 지점)
  * robot이 목표지점까지 이동하는 동안 위치를 update하여 feedback으로 제공
  * robot이 목표지점으로 이동하는 동안에 cancel 가능
  * robot이 최종 목표지점에 도달하면 result message 전송
* turtlesim은 action server를 가지고 있다.
  * action client가 action server에 goal을 전송
  * 이 튜터리얼에서 action /turtle1/rotate_absolute로 action이 무엇인지 또 어떻게 동작하는 것인지에 대해서 알아보았다.

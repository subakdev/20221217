# [nodes 이해하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. ros2 run
   2. ros2 node list
   3. ros2 node info
5. 요약

## 목표
* ROS2에서 제공하는 nodes의 기능 배워보기
* nodes와 상호 작용할 수 있는 tools 배워보기
## 배경지식
### 1. ROS2 graph
* ROS2 graph를 구성하는 ROS2의 핵심 개념들을 익혀보자.
* 정의
  * 한꺼번에 data를 처리하는 ROS2 elements의 네트워크
  * 실행자와 실행자들 간의 연결
  * 시각화 시키기 
### 2. ROS2에서의 nodes
* ROS를 사용하는 robot 내에서 각 node는 하나의 역할을 수행
  * 휠모터 제어
  * LiDAR 제어
  * ...
* 각 node는 다른 nodes들로부터 data를 주거나 받을 수 있다.
  * topic
  * services
  * actions
  * paramters

![](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

* robotics SW란?
  * 각 역할을 수행하는 많은 nodes의 집합

* ROS2에서 하나의 실행자(executable)은 1개 이상의 nodes를 포함할 수 있다! 
## 사전준비
* turtlesim package 설치
* ROS2 환경 source 하기
## 실습
### 1. ros2 run
* ros2 run 명령은 package내의 실행자(executable)를 launch 시킨다. 
```
ros2 run <package_name> <executable_name>
```
* turtlesim을 실행하려면 새로운 터미널을 열고 아래와 같은 명령을 입력한다: 
```
ros2 run turtlesim turtlesim_node
```

* 이전에 봤던 turtlesim windows가 열린다.(이미 이전 튜터리얼에서 본바와 같이)
* 여기서 package name은 turtlesim이고 실행자 이름은 turtlesim_node이다.
* 만약 node name을 모른다면 ros2 node list 명령을 사용하여 node 이름을 찾을 수 있다.
### 2. ros2 node list
* ros2 node list 명령을 실행하면 실행중인 모든 nodes의 이름을 보여준다.
* 이 명령이 node와 상호작용 할때나 많은 nodes이 실행 중이라서 nodes를 추적해야 할 때 유용하다. (먼저 어떤 nodes들이 있는지 알아야 하니까)

* 다른 터미널에서는 아직 turtlesim이 실행 중인 상태이다. 새로운 터미널을 열고 아래의 명령을 입력하자
```
ros2 node list
```

* node name을 출력한다 :
```
/turtlesim
```

* 새 터미널을 열고 teleop node를 아래의 명령으로 구동시킨다 :
```
ros2 run turtlesim turtle_teleop_key
```

* 이번에는 turtlesim package 내에서 turtle_teleop_key라는 실행자를 검색해서 실행시키게 된다.

* ros2 node list를 실행했던 터미널로 가서 동일한 명령을 다시 실행해보자.
* 이제는 실행 중인 2개 nodes 이름이 나타나게 된다.
```
/turtlesim
/teleop_turtle
```

### 2.1 Remapping

* Remapping이란?
  * default node의 속성을 다시 원하는 값으로 할당
    * 기본 node 속성이란
      * node name
      * topic name
      * service name
      * ...
* 마지막 튜토리얼에서 turtle_teleop_key를 remapping 하여 제어할 기본 turtle을 변경한다.
* 마지막 튜터리얼에서는 제어할 기본 turtle을 변경하기 위해서 turtle_teleop_key에서 remapping을 해보자. 

* 이제 /turtlesim node의 이름을 재할당해보자. 새 터미널에서 아래 명령 실행해보자:
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

* turtlesim 에서 다시 ros2 run을 실행하므로 추가로 turtlesim window가 열리게 된다. 
* 하지만 ros2 node lilst를 실행했던 터미널로 돌아가서 동일한 명령을 다시 실행한다면, 3개 node names를 보게 된다.
```
/my_turtle
/turtlesim
/teleop_turtle
```

### 3. ros2 node info

* 이제 nodes의 이름들을 알게 되어서 이 nodes에 대한 정보에 접근할 수 있다 :
```
ros2 node info <node_name>
```

* 가장 최근 node인 my_turtle node를 조사하기 위해서, 아래 명령을 실행한다 :
```
ros2 node info /my_turtle
```
* ros2 node info 명령은 해당 node와 통신하는(ROS graph에서 연결된) subscribers, publishers, services, actions의 목록을 반환한다.
* 다음과 같이 출력된다:

```
/my_turtle
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
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
* /teleopo_turtle 에대해서도 동일한 명령을 실행해보자. 
* my_turtle에 대해서 실행한 것과 연결이 어떻게 다른지 생각해보자.

* 이후 튜터리얼에서 ROS graph concept과 message types 에대해서 좀더 알아보도록 한다.

## 요약
* node는 
  * ROS2 핵심 요소이다
  * robotics 시스템 내에서 단일 모듈과 같은 역할

* turtlesim_node와 turtle_teleop_key 실행자를 실행해서 turtlesim package에서 생성된 nodes를 사용했다.

* ros2 node list 명령으로 활성화된 nodes 이름을 확인했고
* ros2 node info 명령으로 해당 node의 내부를 살펴볼 수 있었다. 
* 이 도구는 복잡한 실제 robot 시스템 내에 data flow를 이해하는데 핵심이 되는 도구이다!
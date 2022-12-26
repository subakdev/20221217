# [services 이해](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. ros2 service list
   3. ros2 service type
   4. ros2 service find
   5. ros2 interface show
   6. ros2 service call
5. 요약

## 목표
* command line 도구를 이용해서 ROS2의 services에 대해서 배워보자.
## 배경지식
* ROS graph에서 nodes들이 서로 통신하는 또 다른 방법
* ROS2 services 구현 모델
  * call-and-response 모델
* ROS2 topics 구현 모델
  * publisher-subscriber 모델
* topics은 nodes가 data streams을 subscribe하고 지속적인 update 가능
* services는 client가 요청할때만 data를 제공

![](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif)

![](https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif)

## 사전준비
* 위에서 언급한 nodes와 topics은 이전 튜터리얼에서 이미 다뤘다. 
* turtlesim package 설치
* 새 터미널 열어서 source 명령 실행
## 실습

### 1. 설정
* /turtlesim, /teleop_turtle 이렇게 2개의 turtlesim 노드를 구동시킨다.
* 새 터미널을 열고 다음 실행 :
```
ros2 run turtlesim turtlesim_node
```
* 다른 새 터미널 열고 다음 실행  :
```
ros2 run turtlesim turtle_teleop_key
```

### 2. ros2 service list
* 새 터미널에서 ros2 service list 명령을 실행하면 현재 활성화된  services들의 목록을 출력한다. :

```
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

* 2개 nodes가 있고 2개의 nodes 모두 동일한 6개의 services의 이름에 parameters가 들어간다.
* 거의 대부분의 ROS2 node는 parameters가 빌드된 구조 services를 가진다. parameter에 대해서는 다음 튜토리얼에서 다룬다. 이 튜터리얼에서는 parameter services에 대해서는 다루지 않는다.

* 자 이제 turtlesim에 한정된 services에 대해서 집중적으로 알아보자. turtlesim 관련 services에는 /clear, /kill, /reset, /spawn, /turtle1/set_pen, /turtle1/teleport_absolute, /turtle1/teleport_relative가 있다.
* 이전 turtlesim과 rqt 사용하기 튜터리얼에서 rqt를 이용하여 나열한 services와 상호작용해봤던 것이 생각날 것이다.

### 3. ros2 service type

* services는 type을 가지고 있다.
  * 이 type에는 service의 request와 response data가 어떻게 구성되는지를 기술하고 있다.
* service types은 topics 타입과 유사하다. 차이점은 service types은 2개의 부분으로 구성된다는 것이다. 하나는 request message이고 다른 하나는 response message이다.

* service의 타입을 찾는 명령은 다음과 같다.: 
```
ros2 service type <service_name>
```

* turtlesim의 /clear service를 살펴보자. 새 터미널에서 다음 명령을 입력한다.:
```
ros2 service type /clear
```

* 출력은 다음과 같다 : 
```
std_srvs/srv/Empty
```

* Empty type의 의미
  * client가 request를 할때 data를 보내지 않고
  * server가 response를 할때도 data를 보내지 않는다.

### 3.1 ros2 service list -t
* 한 번에 모든 활성화된 services의 타입을 보기 위해서 --show-types 옵션을 추가하면 된다. 줄여서 list 명령에 -t 옵션을 추가할 수 있다.
```
ros2 service list -t
```

* 결과 : 
```
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
```

### 4. ros2 service find
* 특정 type의 모든 services를 찾고자 하는 경우 다음 명령을 사용한다 : 
```
ros2 service find <type_name>
```

* 예제로 다음과 같이 명령을 실행하면 Empty 타입의 모든 services를 찾을 수도 있다 : 
```
ros2 service find std_srvs/srv/Empty
```

* 출력 결과 : 
```
/clear
/reset
```

### 5. ros2 interface show
* service type의 구조들여다보기!
* command line으로 services를 호출할 수 있다. 하지만 먼저 입력하는 인자의 구조를 알아야 한다.

```
ros2 interface show <type_name>
```

* /clear services의 타입인 [Empty](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html)에 대해서 명령을 수행하면 : 
```
ros2 interface show std_srvs/srv/Empty
```

* 다음과 같이 출력된다 : 
```
---
```

* --- 는 request와 response 구조를 구분하는 역할을 한다.
* 하지만 이전에 배운것처럼 Empty type은 어떤 data도 주거나 받지 않는다.
* 따라서 Empty type의 구조는 공백이다.

* 이번에는 data를 주고 받는 타입을 가지고 있는 service를 살펴보자. /spawn의 경우 ros2 service list -t 의 결과로 /spawn의 타입은 turtlesim/srv/Spawn이다.

* /spawn call-and-request 내에서 인자를 보기 위해서 다음 명령을 실행한다 : 
```
ros2 interface show turtlesim/srv/Spawn
```

* 다음과 같이 출력된다. : 
```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

* --- 구분자의 윗부분
  * request 시에 필요한 정보
  * /spawn을 call하기 위해서 필요한 인자에 대한 정보이다.
  * x, y, theta가 turtle을 생성하기 위한 위치와 각도를 결정한다.
  * name은 옵션이다.

* --- 구분자 아래 부분
  * response 시에 필요한 부분
  * 내가 request에 대한 응답(response)의 data type 

### 6. ros2 service call
* 지금까지 알아본 내용
  * service type이 무엇인지
  * service type 찾는 방법
  * service type의 내부 구조
 
* 서비스 호출 하는 방법은 다음 명령과 같다. : 
```
ros2 service call <service_name> <service_type> <arguments>
```

* <arguments> 부분은 옵션이다. 예를 들어 Empty 타입의 service는 인자를 갖지 않는다.  : 

```
ros2 service call /clear std_srvs/srv/Empty
```

* 위 명령을 수행하면 turtlesim window에서 turtle의 이동에 따라 그려진 선을 지우게 된다.
![](https://docs.ros.org/en/humble/_images/clear.png)

* 이제 /spawn과 입력 인자를 호출해서 새로운 turtle을 생성해보자. command line에서 service 호출시에 입력 <argument>는 YAML 문법을 이용해야한다.
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

* method-style(함수 호출하고 결과를 보는 방식)을 뷰를 통해 service response 결과를 확인 할 수 있다:

```
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

* turtlesim window에 새로 생성된 turtle을 업데이트 시킨다. :

![](https://docs.ros.org/en/humble/_images/spawn1.png)

## 요약
* ROS2에서 nodes는 services를 통해서도 통신이 가능하다
  * topic
    * 한쪽이 publisher, 다른쪽은 subscriber
  * service
    * request/response 패턴
    * client가 request를 server에게 보낸다. 이 server는 응답으로 response를 보낸다. 
* 만약 어떤 service를 계속해서 call하는 경우라면?
  * topic 이나 action이 적당하다!
* command line 도구를 이용하여 여러 가지 service 관련 동작 수행 연습을 해보았다.

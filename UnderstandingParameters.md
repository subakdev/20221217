# [Parameters 이해하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) (5min)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. ros2 param list
   3. ros2 param get
   4. ros2 param set
   5. ros2 param dump
   6. ros2 param load
   7. node 구동시에 parameter 파일 로드하기
5. 요약

## 목표
* ROS2의 parameters를 get,set,save, reload 하는 방법  익히기 
## 배경지식
* parameter는 node의 설정값이다. node를 setting하는 값으로 생각 할 수 있다. node는 파라미터를 integers, floats, booleans, strings, list로 저장 할 수 있다. ROS2에서는 각 node는 자신의 parameter를 유지하고 있을 수 있다. 자세한 parameter 관련 정보는 [문서](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html) 참고.

## 사전준비
* turtlesim package를 사용한다.

## 실습

### 1. Setup
* /turtlesim, /teleop_turtle 2개 nodes를 실행시킨다 :

새로운 터미널을 열고 실행한다 : 
```
ros2 run turtlesim turtlesim_node
```

새로운 터미널을 열고 실행한다 : 
```
ros2 run turtlesim turtle_teleop_key
```

### 2. ros2 param list

* 해당 node에 속한 parameters를 볼려면, 새로운 터미널을 열고 다음을 실행한다  :

```
ros2 param list
```

* 2개의 node namespaces인 /teleop_turtle과 /turtlesim가 보이고 각 node의 parameters가 출력된다. : 

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

* 모든 node가 use_sim_time 이라는 parameter를 가지고 있다. : turtlesim에 유일하게 사용되는 것은 아니다.
* 이름에서 유추할 수 있듯이 /turtlesim의 parameters는 RGB color 값을 사용하여 turtlesim window의 배경 색상을 결정한다. 

* parameter의 type을 결정하기 위해서 ros2 param set을 사용할 수 있다.

### 3. ros2 param get

* parameter의 type과 현재값을 표시하기 위해서 아래와 같은 명령을 사용한다. : 
```
ros2 param get <node_name> <parameter_name>
```

* /turtlesim의 parameter인 background_g 의 현재 값을 확인해보자. :
```
ros2 param get /turtlesim background_g
```

* 출력되는 값  :
```
Integer value is: 86
```

* 이제 background_g가 정수값을 가지고 있다는 것을 알았다.

* 위외 동일한 명령을 background_r, background_b에도 실행해보면 각각  69, 255 값을 갖는 것을 알 수 있다.

### 4. ros2 param set

* runtime에 parameter 값을 변경하는 명령 : 
```
ros2 param set <node_name> <parameter_name> <value>
```

* /turtlesim의 배경색을 바꿔보는 명령 :
```
ros2 param set /turtlesim background_r 150
```

* 출력 메시지 : 
```
Set parameter successful
```

* turtlesim window의 색상은 아래와 같이 변경되는 것을 볼 수 있다. :

![](https://docs.ros.org/en/humble/_images/set.png)

* set 명령을 통해 parameters를 설정하는 것은 현재 session에서만 반영된다. 즉 다시 turtlesim을 구동시키면 현재 설정한 색상을 유지하지 않는다. 하지만 원한다면 node가 시작될때 마지막에 설정한 값으로 reload되는 것도 가능하다.

### 5. ros2 param dump
* 특정 node에 대한 모든 현재 parameter 값을 보는 명령 :
```
ros2 param dump <node_name>
```

* 명령의 결과는 기본적으로 stdout으로 출력된다. 하지만 parameter 값들을 파일로 저장도 가능하다.
* /turtlesim parameters의 현재 설정값을 turtlesim.yaml 파일로 저장하는 명령 :
```
ros2 param dump /turtlesim > turtlesim.yaml
```

* 동일한 폴더에 turtlesim.yaml 파일이 생성되고 이 파일을 열면 아래와 같다. :
```
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```
 * 나중에 node가 동일한 parameters 값으로 reload시키고자 할때 parameters 값의 덤프를 떠놓으면 편하다.

### 6. ros2 param load

* 파일에 있는 parameters를 현재 실행 중인 node에서 적용하는 명령 :
```
ros2 param load <node_name> <parameter_file>
```

* ros2 param dump 명령으로 생성된 turtlesim.yaml 파일을 /turtlesim node의 parameter에 적용하는 명령 :
```
ros2 param load /turtlesim turtlesim.yaml
```

* 다음과 같은 결과를 출력한다.
```
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
Set parameter use_sim_time successful
```

* 주의 : 
  * 읽기 전용(read-only) parameters는 구동시점(startup)에만 수정이 되고 이후에는 수정이 안된다. 

### 7. node 구동시에 parameter 파일 로드하기
* 저장한 parameter 값을 사용하여 동일한 node를 시작시키기 :
```
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```
* 일반적으로 turtlesim을 구동시킬때 사용하는 명령과 동일하다. 단 --ros-args와 --params-file flags를 추가하고 뒤에 load할 파일이 온다.

* 저장한 parameters로 reload시킬려면 turtlesim node를 중지하고 다음을 실행 : 
```
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

* turtlesim window는 화면에 나타나면서 배경색은 보라색이 적용되어 있다.

* 주의 :
  * 이 경우에, parameters가 node가 시작하는 시점에 적용된다. 따라서 읽기전용(read-only) parameters도 적용이 되는 것이다.

## 요약
* nodes는 기본 설정 값을 정의하는 parameters를 가지고 있다. commnad line에서 parameter 값을 get과 set이 가능하다.
* parameter 설정값을 파일로 저장하여 나중에 node를 reload시에 사용할 수 있다.

# [Action 생성하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html) (5min)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. action 정의하기
   2. action 만들기
5. 요약

## 목표
* ROS2 패키지 내에서 action 정의하기

## 배경지식
* Beginner에서 공부했던 [action 내용](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) 

## 사전준비
* ROS2 설치
* Colcon 설치 
* workspace 설정
* Package 생성( 패키지이름 : action_tutirials_interface)

    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ros2 pkg create action_tutorials_interfaces
    ```

## 실습
### 1. action 정의하기 
* actions은 .action 파일에 정의한다.
* action 기본구조 
    ```xml
    # Request
    ---
    # Result
    ---
    # Feedback
    ```
* action 정의는 3개의 message 정의로 구성되며 구분을 위하여 ---를 사용한다.
  * request : action client가 action server에게 새로운 goal을 전송 
  * result : goal이 완료되면 action server가 action client에게 전송 
  * feedback : action server가 action clinet에게 goal에 대한 업데이트 내용을 주기적으로 전송

* action의 instance를 일반적으로 goal이라고 부른다.

* 새로운 action으로 "Fibonacci"를 정의해 보자.
1) 피보나치 action 정의하기 
  * action 디렉토리 생성
    ```bash
    cd action_tutorials_interfaces
    mkdir action
    ```
  * action 디렉토리 내부에 Fibonacci.action 파일 생성 및 내용 
    ```bash
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
* goal request : 계산하고자 하는 Fibonacci sequence의 order
* result : 마지막 sequence
* feedback : 계산된 partial_sequence
 
### 2.action 빌드하기 

코드에서 새로운 action을 사용하기 위해서는 "rosidl" 코드 생성 파이프라인에 전달해야 합니다.

[rosidl 참고 링크](https://docs.ros.org/en/rolling/Concepts/About-Internal-Interfaces.html#the-rosidl-repository)

1) CMakeLists.txt 적용하기 :
```bash
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

2) package.xml에 적용하기 : 
```bash
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

3) build 하기 : 
```bash
cd ~/ros2_ws
colcon build
```

4) command line 도구로 action 확인하기

action은 패키지/action/action정의이름 형태로 설정된다.
command line tool을 사용하여 정상 빌드되었는지 확인 가능하다.


```bash
. install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

## 요약
* action 정의 및 구조에 대해서 알아보았다.
* CMakeLists.txt와 package.xml을 사용하여 새로운 action interface를 빌드하는 방법을 배웠다.
* 성공적으로 빌드가 수행하였는지 확인하는 방법을 배웠다.

# [Composition에 대해서](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
1. ROS1 - Nodes vs. Nodelets
2. ROS2 - Unified API
3. Component 작성하기
4. Components 사용하기
5. 실제 적용

## 1. ROS1 - Nodes vs. Nodelets
* ROS1에서 코드 작성시 ROS node나 ROS nodelet으로 작성 가능
* ROS1 node는 실행자로 빌드된다. ROS1 nodelets은 shared library로 빌드된다. container에서 runtime시에 load된다. 
## 2. ROS2 - Unified API
* ROS2에서 code를 작성하는 추천방식은 nodelet처럼 작성하는 것이다. 이것을 'Component'라고 부른다.
* 기존 code에 공통 개념을 추가하는 것을 쉽게할 수 있다. 

* Note
  * 물론 기존 node 스타일로 작성은 가능하지만 일반적으로 추천하지 않는다!

* deploy
  * 별도 processes에서 여러 nodes 실행
    * 특정 process가 죽어도 다른 쪽에 영향을 주지 않는다.
    * 개별 node에 대해서만 디버깅하면 된다.
  * 하나의 process 내에서 여러 nodes를 실행
    * 오버헤드가 적다.
    * 효과적인 통신(Intro Process Communication)

## 3. Component 작성하기
* component는 shared library로 빌드된다.
    * main 함수가 없다.
    * rclcpp::Node의 subclass이다.
    * thread의 제어에 들어가지 않기에 길게 실행되지 않아야 하고 생성자에서 다른 tasks를 블랙킹하면 안된다.
    * 대신에 timer를 사용하여 주기적으로 noti를 얻는다. 
    * 추가로 publisher, subscriber, servers, clients를 생성할 수 있다.

* 이런 class를 componet로 만들 수 있는 중요한 요소는 rclcpp_components package에서 제공하는 macro를 사용하여 class가 자신을 등록시킬 수 있다. (소스 코드의 마지막 라인 참고)
* 등록을 시키면 library를 실행 중인 process로 로드시킬때 해당 component를 discoverable상태로 만들어 준다. - entry point로 사용 가능하게 만든다.

* 추가로 일단 component가 생성되면 도구로 discoverable하게 만들기 위해서 index로 register되어야만 한다.
```cmake
add_library(talker_component SHARED
   src/talker_component.cpp)
rclcpp_components_register_nodes(talker_component "composition::Talker")
# To register multiple components in the same shared library, use multiple calls
# rclcpp_components_register_nodes(talker_component "composition::Talker2")
```

* Note
  * component_container 가 원하는 components를 찾도록 하기 위해서는 관련 workspace에서 source한 shell에서 반드시 실행하거나 launch 시켜야 한다.

## 4. Components 사용하기
* composition package에는 components를 사용하는 몇 가지 접근법을 포함하고 있다.
* 가장 일반적인 3가지 방법
   1. generic container process를 구동시키고 container가 제공하는 ROS service load_node를 호출한다.  ROS service는 다음으로 전달받은 package name과 libary로 지정한 component를 load하게 된다. 그리고 실행 중인 process 내부에서 이를 실행시키게 된다. 코드로 ROS service를 호출하는 대신에 command line을 사용하여 전달받은 command line 인자를 가지고 ROS service를 호출한다.
   2. 여러 가지 nodes를 포함하고 있는 커스텀 실행자를 컴파일로 생성한다. 이 접근법은 각 component가 헤더 파일을 가지고 있어야 한다.
   3. launch 파일을 생성하고 ros2 launch를 이용하여 container process를 생성하여 여러 components를 로드시킨다.

## 5. Composition 데모
* [링크](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
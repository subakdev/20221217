# [간단한 publisher와 subscriber 작성하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. publisher node 작성하기
   3. subscriber node 작성하기
   4. 빌드 및 실행하기
5. 요약

## 목표
* C++을 사용하여 publisher와 subscriber node를 생성하기
* 생성한 node 실행하기

## 배경지식
* Node는
  * 실행가능한 process
  * ROS graph 상에서 서로 통신이 가능
* 이 튜터리얼에서 nodes의 동작
  * topic으로 string message를 주고 받기
  * talker와 listener
  * 한쪽은 보내고 한쪽은 받는 동작을 의미
* [코드 다운받기](https://github.com/ros2/examples/tree/humble/rclcpp/topics)

## 사전준비
* 이전 튜터리얼에서 배운 workspace 생성하기와 package 생성하기에 대한 이해

## 실습
### 1. package 생성하기
* 새로운 터미널 열기
* ROS2 환경 source 하기
* ros2 명령 실행되는지 확인하기

* 이전 튜터리얼에서 생성한 ros2_ws 디렉토리로 이동
* 이전에 배운 내용 기억하기
  * src 디렉토리 내에 packages를 생성하였다. 
  * 주의 : workspace 디렉토리 아래가 아니라 src 아래에서 작업을 수행해야함!!!
* ros2_ws/src로 이동하여 package 생성 명령 실행
```
ros2 pkg create --build-type ament_cmake cpp_pubsub
```
* cpp_pubsub과 기타 필요한 파일과 디렉토리 생성되었다는 메시지를 보여준다.
* ros2_ws/src/cpp_pubsub/src로 이동해보자. 빌드할 소스 파일들이 실제로 들어있는 디렉토리이다.

### 2. publisher node 작성하기
* 아래 명령을 실행하여 예제 talker code를 다운받자

```
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
```

* publisher_member_function.cpp 이라는 새로운 파일을 생성하고 내용은 아래와 같이 입력한다.
```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### 2.1 code 살펴보기
* #include
  * rclcpp/rclcpp.hpp : ROS2 시스템이 제공하는 기능 사용 
  * std_msgs/msg/string.hpp : ROS2가 제공하는 message type 사용

* 아래 코드는 node의 의존성. package.xml과 CMakeLists.txt에 이 의존성이 추가되어야 함.
```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```

* 아래 코드는 rclcpp:Node를 상속한 MinimalPublisher class. 여기서 모든 this는 해당 node를 가리킨다.
```c++
class MinimalPublisher : public rclcpp::Node
```
* 
### 2.2 의존성 추가하기
* ros2_ws/src/cpp_pubsub 디렉토리로 이동
* package.xml 파일 열기
* '<description>', '<maintainer>', '<license>' tags 내부 채우기
```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
* ament_cmake buildtool 의존성 뒤에 아래 코드를 복사하여 붙여넣자.
```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

* 코드가 실행될때 rclcpp와 std_msgs가 필요하다는 것을 선언하는 것이다.
* 파일을 저장한다.

### 2.3 CMakeLists.txt
* CMakeLists.txt 파일 열기. 
* 기존 의존성 find_package(ament_cmake REQUIRED) 밑에 아래 코드 추가하기
```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
* talker라는 이름의 실행자를 추가하여 나중에 ros2 run 명령으로 실행이 가능하다.
```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

* 마지막으로 ros2 run 명령이 실행자를 찾을 수 있게 install(TARGETS…) 섹션을 추가한다.
```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

* CMakeLists.txt 내부에 불필요하나 섹션이나 코멘트를 지우기면 아래와 같이 된다.
```
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

* 이제 작성한 package를 빌드하고 실행할 수 있는 준비가 되었다. 전체 시스템이 동작하는 것을 보기 위해서 subscriber node를 먼저 생성하도록 하자.

### 3. subscriber node 작성하기
* ros2_ws/src/cpp_pubsub/src 로 이동하여 다음 node를 생성해보자. 
* 아래 명령을 입력한다.
```
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp
```
* ls 명령을 입력하면 다음과 같이 출력된다.
```
publisher_member_function.cpp  subscriber_member_function.cpp
```
* VSC로 subscriber_member_function.cpp 파일을 열어보자.
```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 3.1 코드 살펴보기
* subscriber 코드는 publisher 코드와 거의 동일하다.
* node의 이름 : minimal_subscriber
* 생성자에서 node의 create_subscription class를 사용하여 callback을 수행한다.
* subscriber는 timer가 필요없다! 왜냐하면 topic에서 message를 수신하면 바로 처리하므로!
```c++
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```
* publisher와 subscriber에서 사용하는 topic의 이름과 message type이 일치해야 서로 통신이 가능하다.
* topic_callback 함수가 하는 일
  * publish되는 string 타입 message를 수신
  * RCLCPP_INFO 매크로를 사용하여 화면에 출력
* 이 class내에서 유일한 field 선언은 subscription이다.
```c++
private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```
* main 함수 publisher와 거의 동일
  * MinimalSubscriber node를 spin
  * Publisher에서는 timer를 구동시켰으나 여기서는 메시지 수신을 대기
* 이 node는 publisher node와 동일한 의존성을 갖고 있어서 별다른 설명이 필요없다.

### 3.2 CMakeLists.txt
* CMakeLists.txt 다시 열고 subscriber node를 위한 실행자와 target를 추가한다. (위치는 publisher 밑에 )
```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```
* 파일을 저장하면 이제  pub/sub 시스템을 사용할 준비가 되었다.

### 4. 빌드 및 실행하기
* rclcpp와 std_msgs는 ROS2 system에 이미 설치되어 있다.
* ros2_ws 디렉토리에서 rosdep를 실행하여 빠진 의존성이 있는지 검사한다. (아래 명령)
```
rosdep install -i --from-path src --rosdistro humble -y
```
* ros2_ws 디렉토리에서 새로운 package를 빌드한다.
```
colcon build --packages-select cpp_pubsub
```
* 새 터미널을 열어서 ros2_ws 디렉토리로 이동해서 setup 파일을 source 한다.
```
. install/setup.bash
```

* 자 이제 talker node를 실행하자.
```
ros2 run cpp_pubsub talker
```

* 터미널은 0.5초마다 message를 publish하기 시작한다.
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```
* 새 터미널을 열어서 위와 같이 source 하고 listener를 구동시킨다.
```
ros2 run cpp_pubsub listener
```

* listener는 console에 message를 출력한다. 
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

* ctrl+c를 눌러서 node 실행을 중단할 수 있다.

## 요약
* 2개의 nodes를 생성했다.
  * publisher
  * subscriber
* 컴파일 및 실행하기 전에 package 설정 파일에 의존성과 실행자를 추가해줬다.
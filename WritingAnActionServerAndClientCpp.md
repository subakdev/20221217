# [Action 서버와 클라이언트 작성 (C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-server-and-client-c)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. action_tutorials_cpp package 생성하기
   2. action server 작성하기
   3. action client 작성하기
5. 요약

## 목표
* action 서버와 클라이언트 구현하기 (C++)

## 배경지식
* action은 ROS에서 비동기 통신 형태이다.
* action client는 goal request를 action server에게 보낸다.
* action server는 goal feedback과 result를 action client에게 보낸다.

## 사전준비
* 이전 튜터리얼에서 정의한 것들이 필요하다. 
    * action_tutorial_interfaces 패키지
    * Fibonacci.action
* [Action 생성하기](./CreatingAnAction.md) 이용

## 실습
### 1. action_tutorials_cpp 패키지 만들기 

###  1.1 action_tutorials_cpp 패키지 만들기

* 이전 튜터리얼에서 만든 action workspace로 이동하여 source 명령 수행한다.
* C++ action server을 위한 새로운 package를 아래 명령으로 생성한다.
  ```bash
  cd ~/ros2_ws/src
  ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
  ```

###  1.2 visibility control에 추가하기 (Windows에서 작업하는 경우)
* Windows에서 package를 빌드하고 실행하는 경우 "visibility control"에 추가 작업이 필요하다.
* 자세한 내용 [링크](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Windows-Tips-and-Tricks.html#windows-symbol-visibility) 확인

  action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h 를 생성하고
  아래의 코드를 넣는다 : 

 ```cpp
 #ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
 #define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

 #ifdef __cplusplus
 extern "C"
 {
 #endif

 // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
 //     https://gcc.gnu.org/wiki/Visibility

 #if defined _WIN32 || defined __CYGWIN__
 #ifdef __GNUC__
     #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
     #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
 #else
     #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
     #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
 #endif
 #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
     #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
 #else
     #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
 #endif
 #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
 #define ACTION_TUTORIALS_CPP_LOCAL
 #else
 #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
 #define ACTION_TUTORIALS_CPP_IMPORT
 #if __GNUC__ >= 4
     #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
     #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
 #else
     #define ACTION_TUTORIALS_CPP_PUBLIC
     #define ACTION_TUTORIALS_CPP_LOCAL
 #endif
 #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
 #endif

 #ifdef __cplusplus
 }
 #endif

 #endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
 ```

### 2. action server 작성
* [action 생성하기](./CreatingAnAction.md) 튜터리얼에서 생성한 action을 사용하여 Fibonacci 순열을 계산하는 action server를 작성해보자. 

### 2.1 action server 코드 작성하기
* action_tutorials_cpp/src/fibonacci_action_server.cpp 파일 열고 다음 코드를 넣기 : 

 ```cpp
 #include <functional>
 #include <memory>
 #include <thread>

 #include "action_tutorials_interfaces/action/fibonacci.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_action/rclcpp_action.hpp"
 #include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

 namespace action_tutorials_cpp
 {
 class FibonacciActionServer : public rclcpp::Node
 {
 public:
 using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
 using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

 ACTION_TUTORIALS_CPP_PUBLIC
 explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
 : Node("fibonacci_action_server", options)
 {
     using namespace std::placeholders;

     this->action_server_ = rclcpp_action::create_server<Fibonacci>(
     this,
     "fibonacci",
     std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
     std::bind(&FibonacciActionServer::handle_cancel, this, _1),
     std::bind(&FibonacciActionServer::handle_accepted, this, _1));
 }

 private:
 rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

 rclcpp_action::GoalResponse handle_goal(
     const rclcpp_action::GoalUUID & uuid,
     std::shared_ptr<const Fibonacci::Goal> goal)
 {
     RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
     (void)uuid;
     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }

 rclcpp_action::CancelResponse handle_cancel(
     const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 {
     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
     (void)goal_handle;
     return rclcpp_action::CancelResponse::ACCEPT;
 }

 void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 {
     using namespace std::placeholders;
     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
     std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
 }

 void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
 {
     RCLCPP_INFO(this->get_logger(), "Executing goal");
     rclcpp::Rate loop_rate(1);
     const auto goal = goal_handle->get_goal();
     auto feedback = std::make_shared<Fibonacci::Feedback>();
     auto & sequence = feedback->partial_sequence;
     sequence.push_back(0);
     sequence.push_back(1);
     auto result = std::make_shared<Fibonacci::Result>();

     for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
     // Check if there is a cancel request
     if (goal_handle->is_canceling()) {
         result->sequence = sequence;
         goal_handle->canceled(result);
         RCLCPP_INFO(this->get_logger(), "Goal canceled");
         return;
     }
     // Update sequence
     sequence.push_back(sequence[i] + sequence[i - 1]);
     // Publish feedback
     goal_handle->publish_feedback(feedback);
     RCLCPP_INFO(this->get_logger(), "Publish feedback");

     loop_rate.sleep();
     }

     // Check if goal is done
     if (rclcpp::ok()) {
     result->sequence = sequence;
     goal_handle->succeed(result);
     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
     }
 }
 };  // class FibonacciActionServer

 }  // namespace action_tutorials_cpp

 RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
 ```

첫 줄에는 컴파일 해야 하는 헤더가 포함 되어있다.
```cpp
#include <functional>
...
```

다음으로 rclcpp:Node를 상속받은 클래스를 생성한다 

```cpp
class FibonacciActionServer : public rclcpp::Node
```

FibonacciActionServer 클래스의 생성자는 노드 이름을 fibonacci_action_server로 초기화 합니다.

```cpp
explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
 : Node("fibonacci_action_server", options) 
```

생성자는 새 작업 서버를 인스턴스화 한다
```cpp
this->action_server_ = rclcpp_action::create_server<Fibonacci>(
    this,
    "fibonacci", 
    std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
    std::bind(&FibonacciActionServer::handle_cancel, this, _1),
    std::bind(&FibonacciActionServer::handle_accepted, this, _1)):
```

* action 서버 구현에 필요한 6가지 
   1. template의 action 타입 이름 : Fibonacci
   2. action을 추가하기 위한 ROS2 node : this
   3. action 이름 : 'fibonacci'
   4. goal 처리를 위한 callback 함수 :  handle_goal
   5. cancellation 처리를 위한 callback 함수 : handle_cancel
   6. goal accept 처리를 위한 callback 함수 : handle_accept

* 여러 callback 구현에 대해서 알아보자. callback 구현에서 중요한 것은 바로바로 return해줘야 한다는 것이다. 그렇지 않으면 executor가 starving하게되는 위험이 있다.

* 새로운 goal을 처리하는 callback 구현 (handle_goal)
```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Reveived goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```
* 이 구현에서는 모든 goals를 accept한다.

* 다음은 cancellation 처리하는 callback 구현 (handle_cancel)
```cpp
rclcpp_action::CancelResponse handle_cancle(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse:ACCEPT;
    }
```
* 이 구현은 client에게 cacenl을 accept했다고 알려준다.

* 마지막 callback은 새로운 goal을 accept하고 처리 시작을 구현 (handle_accepted)

```cpp
void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    using namespace std::placeholders;
    // 쓰레드를 통해 빠르게 return하여 executor의 블로킹을 방지 한다. 실제 처리는 thread가 execute()를 통해서 처리
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
}
```
* 이 실행(execution)은 오래동안 실행되는 작업이므로 실제 작업을 위해서 thread를 생성하여 맡기고 바로 return한다.

* 새 thread 내부의 execute method에서 실제적인 처리와 업데이트 작업이 일어난다.
```cpp
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
```
* 이 work thread는 1초마다 Fibonacci 순열의 하나씩 처리한다. 각 단계에서 feedback update를 publish한다. 그리고 처리를 마치는 시점에 goal_handle를 '성공'으로 표시하고 종료한다.

* 자 이제 완전히 동작하는 action server가 되었으니 이제부터는 빌드하고 실행해보자!

### 2.2 action 서버 컴파일하기
* 이전 섹션에서 action server 코드를 넣었으니 컴파일하고 실행하기 위해서 몇 가지 추가적인 작업이 필요하다.

* 먼저 action server를 컴파일 하려면 CMakeLists.txt 를 설정해야 한다. 
* action_tutorials_cpp/CMakeLists.txt 를 열고 find_package 호출 바로 뒤에 아래 내용을 추가해 보자.

```cmake
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

* 자 이제 package를 컴파일할 수 있다. ros2_ws 디렉토리로 가서 다음 명령을 실행하자.
```
colcon build
```
* 위 명령을 수행하면 전체 workspace를 컴파일하게 된다. (action_tutorials_cpp package 내에 있는 fibonacci_action_server를 포함한 전체)

### 2.3 action server 실행하기 
* 이제 action server가 빌드되었으니 실행할 수 있게 되었다. 금방 빌드한 workspace에 대한 source 명령을 수행하고 action server를 실행해보자.

```bash
ros2 run action_tutorials_cpp fibonacci_action_server
```

### 3. action 클라언트 작성
### 3.1 action 클라언트 코드 작성 
* action_tutorials_cpp/src/fibonacci_action_client.cpp 파일을 열고 다음 내용을 넣자.:

```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
```

첫 몇줄은 컴파일에 필요한 헤더 include.

rclcpp:Node를 상속받은 클래스를 생성한다 

```cpp
class FibonacciActionClient : public rclcpp::Node
```

FibonacciActionClient 생성자가 노드이름을 fibonacci_action_client로 초기화 한다 

```cpp
  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
```

새로운 action 클라이언트를 인스턴스화 한다 
```cpp
this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
    this,
    "fibonacci");
```

* action 클라언트 구현에 필요한 3가지
   1. action template 타입 이름 : Finbonacci
   2. action 클라이언트 추가하기 위한 ROS2 노드 : this
   3. action 이름 : 'fibonacci'

* ROS timer의 인스턴스를 만들어서 send_goal을 호출하도록 구동시킨다.

```cpp
this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&FibonacciActionClient::send_goal, this));
```
* timer가 만료되면 send_goal을 호출한다.
```c++
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
```

* 위 함수가 하는 일 :
   1. timer를 취소한다(그래서 한번만 호출 됨)
   2. action server로부터 보내는 것을 위해 대기한다.
   3. 새로운 Fibonacci::Goal을 생성한다. 
   4. response, feedback 및 result callbacks를 설정한다. (server로부터 받을 event에 대한 callback 설정)
   5. goal를 서버로 전송

* server가 goal을 수신하고 accept하는 시점에 reponse를 client에게 보낸다. 이런 response는 goal_response_callback 에서 처리한다. 
```cpp
  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
```
* server가 이 goal을 받았다고 가정하면, 이제 처리를 시작하게된다. client에게 보내는 feedback은 feedback_callback이 처리한다. :
```c++
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
```

* server가 처리를 마치게 되면 result를 client에게 보낸다. 이 result는 result_callback이 처리한다. :
```c++
  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient
```

* 이제 완전히 동작하는 client를 구현했으니 빌드와 실행을 해보도록 하자.

### 3.2 action 클라이언트 컴파일 및 빌드 
* 이전 섹션에서 action client를 작성했다. 컴파일과 실행을 위해서 해야할 일이 몇가지 있다.

* 먼저 action client를 컴파일 하기 위해서 CMakeLists.txt를 설정해야한다. action_tutorials_cpp/CMakeLists.txt를 열고 find_package 호출 뒤에 바로 아래 내용을 넣는다.

```cmake
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

* 이제 이 package를 컴파일할 수 있다. ros2_ws로 가서 실행하자.
```
colcon build
```

* 위에 명령은 전체 workspace를 컴파일한다. (action_tutorials_cpp package 내에 fibonacci_action_client를 포함한 전체) 

### 3.3 action 클라언트 실행하기
* 이제 action client를 빌드하였으니 실행할 수 있게 되었다.
* 먼저 별도의 터미널에서 action server를 실행한다.
* 위에서 빌드한 ros2_ws workspace에서 source 명령을 수행한다. 그리고 action client를 실행해보자.
```bash
ros2 run action_tutorials_cpp fibonacci_action_client
```
* goal의 수신, feedback, 최종 결과에 대해서 로그 메시지가 출력된다.

```
[INFO] [1671360984.099903393] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55 
[INFO] [1671360985.100391546] [fibonacci_action_client]: Result received: 0 1 1 2 3 5 8 13 21 34 55 
```
## 요약
* 이 튜터리얼에서 C++ action server와 action client의 코드를 살펴봤고 goals, feedback, results를 교환하도록 설정했다.

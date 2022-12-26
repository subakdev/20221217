# Executors
1. 개요
2. 기본 사용법
3. Executors의 types
4. Callback groups
5. Scheduling 의미
6. Outlook
7. 추가 정보

## 1. 개요
* ROS2에서 Execution management(실행 관리)
* Executor
  * 1개 이상의 OS threads 사용
  * incoming message와 events에 대해서 subscriptions, timer, service, action 의 callbacks 호출
* rclcpp에서 제공
  
## 기본 사용법
* 가장 간단한 형태
* main thread가 하는 일
  * incoming messages 처리
  * Node의 events 처리
  * rclcpp::spin() 호출해서 처리 가능
```c++
int main(int argc, char* argv[])
{
   // Some initialization.
   rclcpp::init(argc, argv);
   ...

   // Instantiate a node.
   rclcpp::Node::SharedPtr node = ...

   // Run the executor.
   rclcpp::spin(node);

   // Shutdown and exit.
   ...
   return 0;
}
```

* spin(node)는 기본적으로 single-threaded Executor의 instance 생성하여 node를 실행하게 하는 작업
```c++
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

* Executor의 instance가 spin() 호출하게 되면
  * 현재 thread가 
    * rcl과 middleware에게 혹시 incoming message 있어? event 발생했어라고 계속 querying한다.
  * QoS를 위해서
    * incoming messages를 queue에 저장하지 않고 middleware에 저장
    * callback 함수에서 이를 처리
  * wait set
    * Executor에게 현재 middleware layer에 처리할 messages가 있다는 것을 알림
    * timer expired를 detection 가능
![](https://docs.ros.org/en/humble/_images/executors_basic_principle.png)

* Single-Threaded Executor는 components를 위한 container process로서 사용 가능
  * main 함수 없이도 생성 및 실행이 가능하다는 뜻!

## Executors의 Types
* 현재 rclcpp는 parent를 상속받아 3개 Execuitor types를 제공한다. 
![](https://docs.ros.org/en/humble/_images/graphviz-c1160194dae16051e00be2abef23d0fce5e7c347.png)

* 3개 각 executors는 add_node()를 호출해서 여러 nodes를 사용 가능
```c++
rclcpp::Node::SharedPtr node1 = ...
rclcpp::Node::SharedPtr node2 = ...
rclcpp::Node::SharedPtr node3 = ...

rclcpp::executors::StaticSingleThreadedExecutor executor;
executor.add_node(node1);
executor.add_node(node2);
executor.add_node(node2);
executor.spin();
```

* 위 예제에서 
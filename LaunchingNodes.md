# [nodes 런칭하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. Launch 파일 실행시키기
   2. (옵션) Turtlesim Nodes 제어하기
5. 요약

## 목표
* command line 도구를 이용하여 여러 nodes를 한번에 lauch하기
## 배경지식
* 대부분 튜터리얼에서 새 node를 실행할 떄 새 터미널을 열고 실행시켰다.
* 많은 nodes이 동시에 실행되는 복잡한 시스템을 만드는 경우에 여러 터미널을 열어서 각각 설정을 입력해서 실행해줘야 할 것이다.
* Launch 파일은 ROS2 nodes 를 동시에 설정하여 실행할 수 있다.
* ros2 launch 명령으로 단 하나의 launch 파일을 실행해서 전체 시스템(여러 nodes와 설정)을 한번에 구동시킬 수 있다.

## 사전준비
* 이 튜터리얼을 시작하기전에 ROS2를 설치한다.
* ROS2 humble binary 설치되었다고 가정하여 아래 예제 실행

## 실습
### 1. Launch 파일 실행시키기
* 새 터미널을 열고 실행
```
ros2 launch turtlesim multisim.launch.py
```
* 아래의 launch 파일을 실행하게 된다.
```python
# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```

* 주의
  * 위에 launch 파일은 Python으로 작성되었다. 하지만 XML이나 YAML로도 작성할 수 있다. launch 파일 포맷에 대해서는 이후에 알아보도록 하자.

* 아래와 같이 2개 turtlesim nodes를 실행시킨다.

![](https://docs.ros.org/en/humble/_images/turtlesim_multisim.png)

* 여기서는 launch 파일 내부에 대해서 알아보지 않고 이후에 launch 파일 관련해서 세부적인 내용들을 알아보기로 하자.

### 2. (옵션) Turtlesim Nodes 제어하기
* 이제 이 nodes이 실행되고 있으며, 다른 ROS2 nodes 처럼 제어할 수 있다.
* 예제로 turtles이 반대방향으로 이동하게 만들 수 있다. 아래 2개 터미널을 열어서 실행해보자.

* 2번째 터미널 열어서 실행
```
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

* 3번째 터미널 열어서 실행
```
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
* 이 명령들을 실행하면 결과는 아래 화면과 같다.
![](https://docs.ros.org/en/humble/_images/turtlesim_multisim_spin.png)

## 요약
* 하나의 명령으로 2개 turtlesim nodes를 실행했다.
* 일단 launch 파일을 작성하는 방법을 배웠고, ros2 launch 명령으로 여러 nodes를 설정하여 실행시킬 수 있었다. 
* 보다 상세한 내용은 [launch file 튜터리얼](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) 참고

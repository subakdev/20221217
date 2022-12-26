# [workspace 생성하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. ROS2 환경 source
   2. 새로운 디렉토리 생성하기
   3. 샘플 repo clone하기
   4. 의존성 해결
   5. colcon으로 workspace 빌드하기
   6. overlay를 source하기
   7. overlay를 수정하기
5. 요약

## 목표
* workspace를 생성한다.
* 개발과 테스팅을 위한 overlay 를 설정하는 방법을 배워보자.

## 배경지식
* workspace란?
  * ROS2 packages를 포함하고 있는 디렉토리
* 터미널에서 설치된 ROS2 환경을 source 한다.
  * 설치된 ROS2의 packages를 현재 터미널에서 사용 가능

## 사전준비
* ROS2 설치
* colcon 설치
* git 설치
* turtlesim 설치
* rosdep 설치
* 기본 터미널 사용법
* Visual Studio Code 사용법

## 실습

### 4.1. ROS2 환경 source 하기
* 설치한 ROS2가 이 튜터리얼의 underlay가 된다.
  * underlay는 반드시 설치한 ROS2일 필요는 없다.
* underlay를 source 하기
```
source /opt/ros/humble/setup.bash
```

### 4.2. 새로운 디렉토리 생성하기
* 새로운 workspace를 위한 디렉토리 생성하기
* 이름은 마음대로 지정 가능하며 여기서는 ros2_ws를 사용한다.
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
* 디렉토리 안에 src 디렉토리를 생성한다.
* 위 실행 명령은 ros2_ws 디렉토리 안에 src 디렉토리를 만들고 그 위치로 이동시킨다.

### 4.3. 샘플 repo clone하기
* 터미널 내에서 우선 ~/ros2_ws/src 로 이동한다.
* 여기서 실습해 보는 것은 하나의 workspace를 만들고 기존 packages를 넣어서 빌드해보자.
* 우리가 사용할 기존 packages는 [ros_tutorials 저장소](https://github.com/ros/ros_tutorials/)에 있다. 
* 이미 우리가 경험한 turtlesim도 [이 저장소](https://github.com/ros/ros_tutorials/)에 있는 packages 중에 하나다.
* ros2_ws/src 디렉토리에서 아래 명령을 수행하자.
```
git clone https://github.com/ros/ros_tutorials.git -b humble-devel
```
* Quiz : 위 명령의 의미는 무엇일까?
* ros_tutorials는 workspace에 clone되었다. ros_tutorials 저장소는 turtlesim package도 포함하고 있으며 향후에 한번 사용해 볼 예정이다.
* 이 저장소에 있는 다른 packages는 아직 빌드되지 않은 이유는?
  * COLCON_IGNORE 파일 때문 [링크](https://github.com/ros/ros_tutorials/blob/humble-devel/roscpp_tutorials/COLCON_IGNORE)
* 이제 sample package로 workspace를 채웠지만 아직 완전한 기능을 하는 workspace는 아니다. 먼저 의존성을 해결하고 workspace를 빌드를 우선 시도해보자. 

### 4.4. 의존성 해결
* workspace를 빌드하기 전에 pacakge 의존성을 해결해야 한다. 
* 모든 의존성을 가지고 있을 수도 있지만 좋은 습관은 매번 의존성을 검사하는 것이다. 
* 의존성 때문에 빌드하는 시간이 길어질 수 있기 때문이다.
* ros2_ws workspace에서 다음 명령을 실행한다.
```
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

* 모든 의존성을 가지고 있는 경우라면 다음과 같은 결과를 반환한다.
```
All required rosdeps installed successfully
```

* packages에 대한 의존성은 package.xml 파일에 선언되어 있다.
* 위에 명령으로 선언된 내용들을 보고 만약 없다면 설치하게 된다.

### 4.5. colcon으로 workspace 빌드하기
* ros2_ws에서 아래 명령을 이용하여 빌드한다.
```
colcon build
```
* 결과로 다음과 같은 메시지가 출력된다.
```
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```
* Note
  * colcon build에서 함께 사용하는 유용한 인자들 :
    * --package-up-to
      * 원하는 package를 빌드할때 그 package의 의존성도 함께 빌드한다.(workspace 전체 빌드시키기 싫을때 특정 package만 빌드)
    * --symlink-install
      * python script의 경우 내용이 바뀌어도 path link만 걸어두면 되니까 매번 다시 빌드하지 않아도 된다. 따라서 빌드 시간을 아껴준다. 
    * --event-handlers console_direct+
      * 빌드하는 동안 메시지 출력(이 인자를 사용 안하면 log 디렉토리에 출력이 저장됨)
* 일단 빌드가 끝나면 ~/ros2_ws 내에서 ls 명령을 수행해보자. colcon 명령으로 생성된 디렉토리들을 볼 수 있다.
```
build  install  log  src
```
### 4.6. overlay를 source하기
* overlay를 source 명령 수행하기 전에, 빌드 했던 터미널이 아니라 새 터미널을 열어서 source 명령을 수행한다! 
* 만약 빌드를 했던 동일한 터미널에서 overlay를 source 하면 문제가 발생할 수 있다!!

* 새 터미널에서 ROS2 환경인 underlay에 대해서 source 명령을 수행한다. 
```
source /opt/ros/humble/setup.bash
```

* workspace로 이동
```
cd ~/ros2_ws
```

* worksapce내에서 overlay를 source 하기
```
. install/local_setup.bash
```

* Note
  * overlay의 local_setup은 현재 환경에서 overlay 내에 있는 packages를 사용할 수 있게 추가해주는 역할을 수행한다.
  * setup 은 underlay와 overlay 모두를 사용할 수 있게 source한다.
  * 따라서 ROS2 환경을 setup하고 ros2_ws의 overlay의 local_setup을 source한다. 이것은 ros2_ws의 setup을 source하는 것과 같다. 여기에는 ROS2 환경인 underlay를 source하는 것이 포함되어 있다.
  * 정리 : ros2_ws의 setup source = ROS2 source + ros2_ws local source
* 이제 overlay의 turtlesim package를 실행해보자.

```
ros2 run turtlesim turtlesim_node
```
* 이것은 overlay의 turtlesim 이 실행되는 것이지 ROS2의 turtlesim이 아니다!
* overlay에 있는 turtlesim을 수정하여 확인하기
  * underlay(ROS2 버전) 실행과 overlay(ros2_ws)에서 package를 수정하여 다시 빌드해보자.
  * overlay는 underlay보다 우선한다.

### 4.7. overlay를 수정하기
* 이전에서 실행한 것이 진짜 overlay의 turtlesim인지 확인하는 방법은?
* overlay에 있는 turtlesim을 수정하는데 turtlesim 창의 타이틀바를 수정해보자. 
  * turtle_frame.cpp(~/ros2_ws/src/ros_tutorials/turtlesim/src) 파일을 열기
  * setWindowTitle("TurtleSim")을 "MyTurtleSim"으로 수정하고 저장하기
* colcon build 명령을 실행했던 첫번째 터미널로 돌아와서 다시 colcon build를 실행해보자.
* 두번째 터미널로 가서 turtlesim을 아래와 같이 다시 실행해보자. 
```
ros2 run turtlesim turtlesim_node
```

* 이제 화면에 "MyTitleSim" 타이틀바를 확인할 수 있다.

![](https://docs.ros.org/en/humble/_images/overlay.png)

* 비록 ROS2 환경을 이 터미널에서 source를 했지만 ros2_ws 환경이 underlay 환경보다 우선한다는 것을 알 수 있다.
* underlay가 변경되지 않았따는 것을 확인하기 위해서 새 터미널을 열고 ROS2 환경을 source하고 아래 명령을 실행해보자.
```
ros2 run turtlesim turtlesim_node
```

![](https://docs.ros.org/en/humble/_images/underlay.png)

* 결론 : overlay의 수정은 underlay에 영향을 주지 않는다!


## 요약
* 이 튜터리얼에서 ROS2 humble 설치 버전이 underlay가 된다. overlay는 내가 새로 workspace를 만들어서 package를 추가하여 빌드한 것이 된다.
* overlay는 underlay보다 우선순위가 높다. (path에 대한 우선순위. 즉 먼저 overlay에서 찾고 이후에 underlay에서 실행자를 찾는다.)
* 개발을 할때 overlay를 사용해서 개발하는 것을 추천한다. 그리고 하나의 workspace에 개발하는 모든 packages를 다 넣는 것을 추천하지 않는다.

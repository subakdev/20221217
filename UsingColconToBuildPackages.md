# [Colcon으로 패키지 빌드하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
1. 목표
2. 배경지식
3. 사전준비
   1. colcon 설치
   2. ROS2 설치
4. 기본
   1. workspace 생성하기
   2. 코드 추가하기
   3. underlay를 source하기
   4. workspace 빌드하기
   5. test 실행하기
   6. 해당 환경을 source하기
   7. demo 실행하기
5. 나만의 package 생성하기
6. colcon_cd 설정하기
7. colcon 탭 자동완성 설정하기
8. Tips

## 목표
* 'colcon'을 사용하여 ROS2 workspace 생성 및 빌드하기 실습
* colcon 관련 문서도 있지만 여기서는 실제로 사용하는 부분에 초점을 두고 설명

## 배경지식
* 지금까지 ROS 빌드 도구들
  * catkin_make
  * catkin_make_isolated
  * catkin_tools
  * ament_tools
* colcon은 새삥 빌드 도구! (현재 ROS2에서는 colcon으로 대체되었음)
* [상세정보](https://design.ros2.org/articles/build_tool.html)

## 사전준비
* colcon 설치하기
```bash
sudo apt install python3-colcon-common-extensions
```
* ROS2 설치

## 기본
* ROS workspace
  * 특수한 구조를 가지는 디렉토리
  * src 디렉토리 아래에 실제 ROS2 package 소스코드가 들어간다.
```
src--
    |-- package1
    |-- package2
    |-- package3
    ...
    |-- package-n
```
* colcon으로 소스를 빌드하면 src 디렉토리와 이웃하는 위치에 빌드 결과물이 생성된다.
* colcon으로 생성되는 디렉토리 : build, install, log
```
src     --
build   -- 
install -- 
log     -- 
```
* build : 빌드하면 빌드 중간 결과물이 저장되는 디렉토리
* install : 각 package가 설치되는 디렉토리
* log : colcon 호출될때마다 로그 정보를 저정하는 디렉토리

### 4.1. workspace 생성하기
* ros2_ws 디렉토리 생성
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

* 현재 시점에는 src 디렉토리는 비어있는 상태이다.
```
.
└── src

1 directory, 0 files
```

### 4.2. 소스 추가하기
* workspace의 src 디렉토리에 예제를 clone하기 
```bash
git clone https://github.com/ros2/examples src/examples -b humble
```

* src 디렉토리 안에 소스 코드가 생겼다.
```
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```

### 4.3. underlay를 source하기
* 왜 uderlay를 source 해주어야 하는가?
* 현재 설치된 ROS2 환경에 대해서 source하기
  * 예제 packages를 빌드하는데 의존성을 제공
* source 방법
  1. binary 설치에서 제공하는 setup script를 source 하기 (source /opt/ros/humble/setup.bash)
  2. 이를 underlay라고 부른다. -> 기본 ROS2 설치 환경을 source 하는 환경
* 우리가 작업하고 있는 ros2_ws는 overlay이다.
  * 현재 설치된 ROS2 설치버전의 상위(overlay)에 존재한다.
  * 즉 underlay(ROS2 기본 설치 환경) 위에 내가 작성하는 workspace가 overlay가 된다.
* overlay 사용에 대한 추천 방식
  * 모든 pakcages를 하나의 workspace에 두는 것을 추천하지 않음. 
  * 작은 packages로 구성된 overlay사용
  * 즉 의미있는 작은 단위의 workspace로 구성하고 거기서 필요한 의존성 설정하는 방식으로 코드 작성을 추천!!
  * SW 모듈화 처럼!

###  4.4. workspace 빌드하기
* colcon build 명령 실행하기
  * 항상 내가 작성하는 workspace 디렉토리 아래에서 실행
* colcon은 --symlink-install 옵션을 지원
  * source space 내에서 파일 변경에 의한 변경 발생시 설치된 파일에도 변경시키는 것
  * 왜냐하면 컴파일 되는 변경 부분만 install 디렉토리에 자동 변경되며 보통 python이나 컴파일하지 않는 resource 파일은 변경되어도 install 디렉토리에 업데이트 시키지 않으므로 이 옵션을 쓰도록 하자. 
  * 즉 빌드 -> 파일 변경 -> 변경된 파일 적용
```bash
colcon build --symlink-install
```
* 빌드를 마치면 아래와 같은 디렉토리 구조가 생성된다.
```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

###  4.5. test 실행하기
* 막 빌드한 packages에 대한 테스트를 실행하기 위해서 다음 명령을 수행한다.
```
colcon test
```

### 4.6. 해당 환경을 source하기
* colcon이 빌드를 성공적으로 수행한 후에 결과물은 install 디렉토리에 생성된다. 
* 생성된 실행자나 library를 사용하기 전에 이를 path와 library path에 추가해 놓아야 사용 가능
  * 환경 설정을 위한 bash 파일도 install 디렉토리에 생성된다.
  * 이 bash 파일에는 필요한 path와 library path와 빌드한 package가 제공하는 shell 명령을 사용할 수 있게하는 설정이 되어 있다.
```
. install/setup.bash
```

###  4.7. demo 실행하기
* 나의 개발 환경을 source하면 내가 빌드한 실행자를 실행할 수 있다.
* 예제에서 subscribe node를 실행해 보자.
```
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

* 새로운 터미널을 열어서 publisher node를 실행해보자. (setup 스크립트를 source하는 것을 잊지말자!)
```
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```
* 숫자가 증가하면서 publisher와 subscriber가 주고 받는 것을 볼 수 있다.

## 나만의 package 생성하기
* colcon은 package.xml 파일을 사용
* colcon은 다양한 build type을 제공 (C++, Python 추천)
  * 추천하는 build type
    * ament_cmake : C++
    * ament_python : Python
    * 순수 cmake package도 지원해줌
* ament_python 빌드의 예제
  * [ament_index_python package](https://github.com/ament/ament_index/tree/humble/ament_index_python)
  * setup.py 파일이 빌드를 위한 시작점이다.
* ament_cmake 빌드 타입 예제
  * [demo_nodes_cpp package](https://github.com/ros2/demos/tree/humble/demo_nodes_cpp)
  * CMake 빌드 tool 사용
* 편의를 위해서 ros2 pkg create 도구로 새로운 package를 생성
  * ros2 pkg create 명령을 사용하면 template기반으로 package 생성 가능!

## colcon_cd 설정하기
* colcon_cd 명령
  * 현재 디렉토리를 package의 디렉토리로 바로 전환하는 명령
* 예제
  * colcon_cd_some_ros_package 명령을 수행하면 ~/ros2_install/src/some_ros_package 디렉토리로 이동

```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```
* 위에 path는 자신의 PC에서 설치한 환경에 따라서 조금 달라질 수는 있으니 확인 필요.

## colcon 탭 자동완성 설정하기
* colcon 명령에서 Tab 키로 자동완성 기능
  * colcon-argcomplete package가 설치되어 있어야 한다.
```
colcon l <tab key>
colcon list <자동완성>
```
```
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
* 위에 path는 자신의 PC에서 설치한 환경에 따라서 조금 달라질 수는 있으니 확인 필요.


## Tips
* 특정 package를 빌드하지 않으려면
  * 해당 package 내에 COLCON_IGNORE 라는 빈 파일으르 생성하면 된다.
* CMake packages 내부에서 test 설정 및 빌드를 하지 않게
  * --cmake-args -DBUILD_TESTING=0
* package의 특정 test를 실행하는 명령
```
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```

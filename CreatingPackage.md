# [package 생성하기](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
1. 목표
2. 배경지식
   1. ROS2 package란?
   2. ROs2 package 구성
   3. workspace 내에서의 packages
3. 사전준비
4. 실습
   1. package 생성하기
   2. package 빌드하기
   3. setup 파일을 source 하기
   4. package를 사용하기
   5. package contents를 조사하기
   6. package.xml 커스텀
5. 요약

## 목표
* 새로운 package를 생성
  * CMake나 Python을 이용하기
* 실행자를 실행시키기

## 배경지식
### 1. ROS2 package란?
* ROS2 code의 container라 할 수 있다
* 내가 작성한 code를 설치하거나 다른 사람들과 공유할려면 package 형태로 구조화가 필요하다.
* 나의 ROS2 작업물을 배포할 수 있고 다른 사람이 이를 이용하여 쉽게 빌드하여 사용할 수 있다.
* ROS2 내에서 package 생성에는 ament를 사용한다. 
  * ament : 빌드 시스템
  * colcon : 빌드 도구
* CMake나 Python을 이용하여 package를 생성할 수 있다. 이 2개를 공식 지원하고 다른 빌드 타입은 비공식 지원이다.
### 2. ROS2 package 구성
* ROS2 Python과 CMake package 각각은 반드시 포함해야 하는 contents가 있다 : 
  * CMake
    * package.xml 파일 : packgae에 대한 meta 정보 포함
    * CMakeLists.txt : package 내부에 코드를 어떻게 빌드하는지를 설명
  * Python
    * package.xml : package에 대한 meta 정보 포함
    * setup.py : package를 install하는 방법에 대한 명령어를 포함
    * setup.cfg : package가 실행자들(executables)을 가지는 경우에 필요. 이 파일을 통해서 ros2 run이 실행자들을 찾을 수 있게 된다.
    * /<package_name> : package와 같은 이름을 가지는 디렉토리. ROS2 tool이 package를 찾는데 사용한다.
 
* 가장 간단한 형태의 package 구조
  * CMake
    ```
    my_package/
    CMakeLists.txt
    package.xml
    ```
  * Python
    ```
    my_package/
    setup.py
    package.xml
    resource/my_package     
    ```
### 3. workspace 내부에 packages
* 하나의 workspace는 원하는 만큼의 packages를 포함할 수 있다.
* 하나의 workspace 내부에 있는 각 package는 서로 다른 build types을 가지고 있을 수도 있다. (CMake, Python, 등등. 즉 workspace 내에 packages는 C++로 된 것도 Python으로 된 것도 혼합해서 존재할 수 있다!)
* nested packages를 가질 수 없다!! (package 내부에 또 다른 package를 가지는 구조는 안됨)
* 추천 방식
  * workspace 내부에 src 디렉토리를 갖고
  * src 디렉토리 내에 각 packages를 생성

* 간단한 workspace 구조
```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

## 사전준비 
* 이전 튜터리얼에서 작업한 ROS2 workspace이 있어야 한다.
* 이 workspace 내에 새로운 package를 만들어보자.

## 실습
### 1. package 생성하기
* 이전 튜터리얼에서 만든 ros2_ws를 사용하자.
* 새 package를 생성을 위해서 src 디렉토리로 이동한다.
```
cd ~/ros2_ws/src
```
* 새 package를 생성하는 명령의 기본 형태 문법은 아래와 같다.
```
ros2 pkg create --build-type ament_cmake <package_name>
```
* 처음 튜터리얼에서 --node-name 인자를 사용해서 간단한 Hello World 실행자를 생성한다.
* 터미널에서 다음 명령을 입력하자.
```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```
* worksapce 내에 src 디렉토리에 my_package 라는 디렉토리가 생성될 것이다.
* 명령을 실행한 후에 터미널은 아래와 같은 메시지가 출력된다.
```
going to create a new package
package name: my_package
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source and include folder
creating folder ./my_package/src
creating folder ./my_package/include/my_package
creating ./my_package/CMakeLists.txt
creating ./my_package/src/my_node.cpp
```
* 새 package를 위해서 자동으로 생성되는 파일들을 볼 수 있다.

### 2. package 빌드하기
* workspace 내부에 여러 packages를 넣었을때 얻는 장점
  * colcon build 이라는 빌드 명령을 수행하면 workspace root 아래에 있는 모든 package를 한번에 빌드한다.
  * 그렇지 않으면 각 package 혹은 다른 workspace로 이동하여 빌드를 각각 수행해줘야 한다.

* workspace의 root로 이동하자.
```
cd ~/ros2_ws
```

* 아래 명령으로 빌드해 보자.
```
colcon build
```

* 이전 튜터리얼에서 ros2_ws 내부에 ros_tutorials package가 있었다.
* 여기서 colcon build 명령을 수행하면 turtlesim package도 빌드가 된다.
* 여러 packages가 같이 빌드가 될 것이고 만약에 packages를 더 많이 추가한다면 모든 packages를 빌드하므로 빌드하는데 걸리는 시간이 길어지게 된다.
* 만약에 하나의 package만 빌드하고자 한다면 my_package package만 빌드하는 경우 명령은 다음과 같다.
```
colcon build --packages-select my_package
```

### 3. setup 파일을 source 하기
* 새로운 package와 실행자를 사용하고자 한다면, 먼저 새로운 터미널을 열고 ROS2 환경에 대해서 source 명령을 수행한다.
* 다음으로 ros2_ws 디렉토리 내에서 아래와 같은 source 명령을 수행한다.
```
. install/local_setup.bash
```

### 4. package를 사용하기
* --node-name 인자를 사용하여 생성한 실행자(executable)를 실행시킬려면 아래와 같은 명령을 수행한다.
```
ros2 run my_package my_node
```
* 터미널에 다음과 같은 메시지가 출력된다.
```
hello world my_package package
```

### 5. package contents를 조사하기
* ros2_ws/src/my_package 내부에서 ros2 pkg create 명령을 수행하여 자동으로 생긴 파일과 디렉토리를 볼 수 있다.
```
CMakeLists.txt  include  package.xml  src
```
* src 디렉토리 안에 my_node.cpp가 있다. 모든 커스텀 C++ nodes이 여기에 들어간다.

### 6. package.xml 커스텀
* package를 생성한 후에 반환되는 message에서 description, license, TODO 항목이 있다. 이 부분은 자동으로 설정되지 않으므로 배포판을 만드는 경우 이 부분은 직접 입력을 해야한다.
* ros2_ws/src/my_package에서 package.xml 파일을 열어보자. 
```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <buildtool_depend>ament_cmake</buildtool_depend>

 <test_depend>ament_lint_auto</test_depend>
 <test_depend>ament_lint_common</test_depend>

 <export>
   <build_type>ament_cmake</build_type>
 </export>
</package>
```
* maintainer에서 name, email을 입력하자. description 에도 package에 대한 설명을 추가하자. 
```xml
<description>Beginner client libraries tutorials practice package</description>
```
* license에는 오픈소스에 관한 라이센스를 입력하자 여기서는 Apache License 2.0을 사용한다.
```xml
<license>Apache License 2.0</license>
```
* 저장하기!
* license tag 밑에 _depend로 끝나는 tag 이름들이 있다.
  * package.xml에 의존하는 외부 packages의 목록을 추가한다.
  * rosdep 명령어가 바로 이 _depend tag를 보고 의존성을 확인하게 된다.
* my_package는 간단해서 외부 의존성을 가지지 않았지만 다음 튜터리얼에서 이부분을 추가하여 외부 의존성을 사용하는 것이 일반적이다.

## 요약
* 내가 작성한 code를 구조화하기 위해서 package로 만들어봤다.
* package 생성 명령을 사용하여 필요한 파일들이 자동으로 생성되었다.
* colcon을 사용하여 빌드하였고 로컬 환경에서 실행자를 실행해보았다.

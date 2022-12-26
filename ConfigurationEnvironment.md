# [환경 설정](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. setup 파일을 source 하기
   2. 시작프로그램(shell 구동시)에 source 추가하기
   3. 환경 변수 확인하기
5. 요약

## 목표
* ROS2 개발을 위한 환경 준비

## 배경지식
* ROS2는 shell 환경을 이용하여 여러 workspaces들을 조합하여 실행
* 용어
  * "Workspaces"는 ROS2로 개발하고 있는 location
    * xxx workspaces라고 부르는데 underlay와 overlay가 있다.
  * "underlay"는 core ROS2 workspace (ROS2 설치된 workspace)
  * "overlay"는 이외 local workspace (내가 작성하는 workspace)
* 다른 버전의 ROS2 설치(humble, foxy 등 여러 버전 설치 가능)
  * 각 workspace는 각기 다른 ROS2 버전에서 돌아갈 수도 있다.
  * 이렇게 하기 위해서 unerlay, overlay 개념에 대한 이해 필수!
  * ROS2 환경과 workspace의 setup 파일 source를 이용하여 가능하다. 
```
------------
| overlay  |
------------
| underlay |
------------
```

## 사전준비
* [ROS2 설치](https://docs.ros.org/en/humble/Installation.html) 
  * ROS2 Humble Ubuntu 22.04에서 설치
* 

## 실습
### 1. setup 파일을 source 시키기
* 새 터미널을 열때마다 아래 명령 수행 (ROS2 명령 실행 가능하게 하기 위해)
```bash
source /opt/ros/humble/setup.bash
```

### 2. 시작프로그램에 등록시키기
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. 환경 변수 확인하기
* 위에 source 명령으로 ROS2에서 사용하는 환경 변수들이 설정된다.
* 확인하기
```bash
printenv | grep -i ROS
```
* 결과
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

* 환경변수가 제대로 설정되어 있지 않다면 ROS2 설치를 다시 진행해야함!!

### 3.1 ROS_DOMAIN_ID 변수
* ROS2 agent의 그룹에 대해서 유일한 정수값을 설정할 수 있다. [Domain ID 상세 내용 링크](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
```
export ROS_DOMAIN_ID=<your_domain_id>
```
* 시작프로그램으로 등록
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

### 3.2 ROS_LOCALHOST_ONLY 변수
* 기본적으로 ROS2는 localhost에 한정하지 않음. 즉 네트워크에 있는 다른 PC와 연결이 가능
* 만약 현재 PC에서만 동작하도록 설정하기
  * 즉 네트워크에 다른 PC와 통신(topic pub/sub)하지 않게 하기 위해서 설정
```
export ROS_LOCALHOST_ONLY=1
```
* 시작프로그램으로 등록
```
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

## 요약
* ROS2 환경변수는 개발시작하기 전에 제대로 설정되어 있어야만 한다!
* 설정 방법 (2가지)
  * 매번 터미널 창을 열때마다
  * 시작프로그램으로 등록
* ROS2 관련 명령 실행시 문제가 발생하는 경우
  * 우선 환경변수 설정이 제대로 되어 있는지 확인부터 하자!!!

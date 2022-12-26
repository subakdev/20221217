# [움직이는 로봇 모델 만들기](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html)
1. 목표
1. 머리
1. 집게손
1. 집게손 팔
1. 다른 타입의 조인트들
1. 자세 명시하기

## 목표
* URDF에서 움직이는 조인트 정의하기

이 튜토리얼에서는, 이전 튜토리얼에서 만들었던 R2D2 모델을 조인트가 움직일 수 있도록 수정할 것이다. 이전 모델에저, 모든 조인트는 고정이었다. 이제 우리는 세가지 중요한 조인트 타입을 살펴 볼 것이다: continuous, revolute, prismatic

진행하기 전에 모든 사전 준비단계가 설치되었는지 확인하자. 어떤 것들이 필요한지는 이전 튜토리얼에 나와있다.

이 튜토리얼에서 나오는 모든 모델은 [urdf_tutorial](https://index.ros.org/p/urdf_tutorial)에서 찾을 수 있다.

[여기](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/06-flexible.urdf)에 유연한 조인트를 사진 새로운 urdf가 있다. 어떤 것들이 바뀌었는지 이전 버전과 비교해 볼 수 있지만, 우선 세가지 조인트 예제를 우선적으로 살펴보자.

이 모델을 시각화 하고 제어하기 이해, 이전 튜토리얼과 같이 아래 명령어를 실행해보자.

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf
```

이번에는 비고정 조인트의 값을 제어할 수 있는 GUI가 팝업될 것이다. 모델이 어떻게 움직이는지 조정해 보자. 그런 다음, 어떻게 우리가 이렇게 만들 수 있을지 살펴볼 것이다.

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/flexible.png)

## The Head

```xml
<joint name="head_swivel" type="continuous">
  <parent link="base_link"/>
  <child link="head"/>
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0.3"/>
</joint>
```

몸통과 머리의 연결은 continuous 조인트로 이루어져 있는데, 이는 음의 무한대에서 양의 무한대 까지 어떤 각도든 가능하다는 것을 의미한다. 바퀴 또한 양쪽 방향으로 끝없이 굴러갈 수 있도록 이런 식으로 모델링 된다.

한가지 추가해야 할 점은 xyz 세 쌍으로 명시되는 회전 축이고, 이것들은 머리가 회전할 벡터를 특정한다. 우리는 머리가 z축 기준으로 회전하길 원하므로, 벡터를 "0 0 1"로 정한다.

## The Gripper

```xml
<joint name="left_gripper_joint" type="revolute">
  <axis xyz="0 0 1"/>
  <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
  <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
  <parent link="gripper_pole"/>
  <child link="left_gripper"/>
</joint>
```

오른쪽과 왼쪽 집게손 조인트 둘다 revolute 조인트들로 모델링 되었다. 이는 조인트들이 continuous 조인트처럼 회전하지만 엄격한 한계를 가진다는 것을 의미한다. 때문에, 우리는 조인트의 상/하한(라디안)을 명시하는 limit 태그를 포함해야 한다. 최대 속도와 힘도 명시해야 하지만, 현재 우리의 목표를 위해서는 실제값은 필요 없다.

## The Gripper Arm

```xml
<joint name="gripper_extension" type="prismatic">
  <parent link="base_link"/>
  <child link="gripper_pole"/>
  <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
  <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
</joint>
```

집게손 팔은 prismatic 조인트라는 다른 종류의 조인트이다. 이는, 한 축을 기준으로 회전하는 것이 아니라 축을 따라 움직인다는 것을 의미한다. 이 병진 운동은 로봇 모델이 집게손 팔을 늘리거나 줄일 수 있게 해준다.

prismatic 팔의 한계깞은 revolute 조인트와 같고, 단위만 radian대신 미터로 바뀐다.

## 다른 타입의 조인트들
공간을 움직이는 두 가지 다른 종류의 조인트가 있다. prismatic 조인트는 1차원으로만 움직일 수 있는 반면, planar 조인트는 평면, 즉 2차원으로 움직일 수 있다. 게다가, floating 조인트는 제한없이 3차원 공간을 자유롭게 움직일 수 있다. 이 조인트들은 숫자 하나로 명시될 수 없기 때문에 이번 튜토리얼에서는 다루지 않는다.

## 자세 명시하기
GUI의 슬라이더를 움직이면, Rviz상에서 모델이 움직인다. 이것이 가능케 하는 과정은, 첫 번째로 GUI가 URDF를 파싱해서 모든 비 고정 조인트와 그 한계값을 찾는다. 그리고, sensor_msgs/msg/JointState 메세지를 publish하기 위해 슬라이더 값을 사용한다. robot_state_publisher는 다른 부품들 사이에 모든 transform을 계산하기 위해 이 메세지들을 사용한다. 산출된 transform tree는 Rviz상에서 모든 형상들을 디스플레이하기 위해 사용된다.
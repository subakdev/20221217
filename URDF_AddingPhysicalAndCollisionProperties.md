# [물리 및 충돌 속성 추가하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)
1. 목표
1. 충돌
1. 물리 속성
1. 다른 태그들

## 목표
* 링크에 충돌 및 관성 속성 추가하는 방법 학습하기
* 조인트에 조인트 동역학을 추가하는 방법 학습하기

이번 튜토리얼에서는, 몇가지 기본 물리 속성들을 URDF에 추가하는 방법과 충돌 속성을 명시하는 법을 살펴볼 것이다.

## 충돌
지금까지, 우리는 로봇의 겉모습을 정의하는 서브 엘리먼트, visual만으로 링크들을 명시해왔다. 로봇을 시뮬레이션 하거나 충돌 감지를 위해서는 collision 엘리먼트를 정의할 필요가 있다. [여기](https://raw.githubusercontent.com/ros/urdf_tutorial/master/urdf/07-physics.urdf)에 충돌 및 물리 속성들을 가진새로운 urdf가 있다.

아래는 새로운 base link에 대한 코드이다.

```xml 
<link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 .8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
  </link>
```

* 충돌 엘리먼트는 링크 오브젝트의 바로 하위에 있는 서브 엘리먼트이며, 비주얼 태그와 같은 레벨이다.
* 충돌 엘리먼트는 visual 엘리먼트처럼 geometry 태그로 형태를 정의한다. geometry 태그의 포멧은 visual에서와 동일하다.
* 또한 collision 태그의 서브 엘리먼트에서처럼 원점을 명시할 수 있다.

대부분의 경우에서, collision geometry와 원점이 visual geometry 및 원점과 같기를 원한다. 하지만 그럴 수 없는 두 가지의 케이스가 있다.

* 빠른 처리 속도 : 두개의 간단한 geometry가 아닌, 두개의 메쉬에서의 충돌 감지는 큰 계산 복잡도를 가진다. 때문에, collision 엘리먼트에서 메쉬를 더 간단한 geometry로 바꿀 필요가 있는 경우가 있다.
* 안전지대 : 민감한 장비의 움직임을 제한해야 하는 경우가 있다. 예를 들어, R2D2의 머리에 아무 충돌도 없기를 바란다면, 머리에 근접하는 물체가 없도록 머리를 감싸는 원기둥을 collision geometry로 정의할 수 있다.

## 물리 속성
gazebo와 같은 물리엔진이 모델을 적절히 시뮬레이션 하기 위해서, 정의가 필요한 몇가지 물리적 속성이 있다.

### 관성
모든 시뮬레이션 되는 링크 엘리먼트는 관성 태그를 가진다. 아래에 간단한 예가 있다.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1e-3" ixy="0.0" ixz="0.0" iyy="1e-3" iyz="0.0" izz="1e-3"/>
  </inertial>
</link>
```

* 이 엘리먼트 또한 링크 오브젝트의 서브 엘리먼트이다.
* 질량은 kg으로 정의된다.
* 3x3 회전 관성 행렬도 관성 엘리먼트에 명시된다. 이것은 대칭행렬이므로 다음과 같이 6개의 요소로만 표현된다. 

| ixx | ixy | ixz |
| --- | --- | --- |
| **ixy** | **iyy** | **iyz** |
| **ixz** | **iyz** | **izz** |

* 이 정보는 MeshLab과 같은 모델링 프로그램을 이용해 구할 수 있다. 기하학적 primitives(원기둥, 박스, 구)의 관성은 위키피디아에 [관성 텐서 모멘트 리스트](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)를 사용해서 계산될 수 있다(위 예제에서 사용되었음).
* 관성 텐서는 질량 및 질량분포에 의존한다. 물체의 부피에 따른 질량분포가 같다고 가정하고, 위에서 설명한것 처럼 모양 기반으로 관성 텐서를 계산하는 것도 적당한 근사치를 구하는 방법 중 하나이다.
* 구하기 힘들다면, ixx/iyy/izz에 0.01(한 변이 0.1m이고 0.6kg의 질량을 가진 정육면체와 일치함)이하의 값을 넣는 것도 합리적인 방법이다. 단위 행렬(한 변이 0.1m이고 600kg의 질량을 가진 정육면체와 일치함)로 선택하는 것은 그리 좋은 선택이 아니다.
* 또한 무게중심과 관성 프레임(링크 프레임을 기준으로 하는)을 정의하기 위한 원점을 명시할 수 있다.
* 실시간 제어기를 사용할 때, 관성 엘리먼트가 0이면(또는 0에 근접), 로봇 모델이 경고없이 붕괴되거나 모든 링크들의 원점이 world의 원점과 겹치는 현상이 나타날 수 있다.

### 마찰 계수
링크들이 서로 부딪힐 때, 어떻게 행동할 지 정의할 수 있다. 이는 contact_coefficients라는 collision 태그 서브엘리먼트를 통해 가능하다. 명시해야할 속성은 3가지가 있다:

* mu - [마찰 계수](https://simple.wikipedia.org/wiki/Coefficient_of_friction)
* kp - [강성 게수](https://en.wikipedia.org/wiki/Stiffness)
* kd - [감쇠 계수](https://en.wikipedia.org/wiki/Damping_ratio#Damping_ratio_definition)

```xml
<collision>
    ...
    <contact_coefficients mu="1.0" kp="1.0" kd="1.0" />				
    ...
</collision>
```

### 조인트 동역학
조인트가 어떻게 움직이는 지에 대한 정의는 조인트에대한 dynamics 태그에 의해 정의 된다. 명시해야할 속성은 2가지가 있다:

* friction - 물리적 정지 마찰력. prismatic 조인트의 경우, 단위는 뉴턴이다. 회전 조인트의 경우, 단위는 뉴턴 미터이다.

* damping - 물리적 감쇠 값. prismatic 조인트의 경우, 단위는 미터당 뉴턴 초이다. 회전 조인트의 경우 라디안당 뉴턴 미터 초이다.

```xml
 <joint>
    ...
    <dynamics damping="0.0" friction="0.0"/>
    ...
 </joint>
```

명시하지 않으면, 기본값은 0이다.

## 다른 태그들
조인트를 정의하는데 필요한 URDF만을 위한(gazebo용 태그를 제외한) 2개의 태그가 있는데, calibration과 safety이다. 이번 튜토리얼에서는 다루지 않으므로, [스펙](https://wiki.ros.org/urdf/XML/joint)을 확인해보자.

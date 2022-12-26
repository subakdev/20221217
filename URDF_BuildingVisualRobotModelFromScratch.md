# [비주얼 로봇 모델 만들기](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html#origins)
1. 목표
1. 한가지 모양
1. 여러가지 모양
1. 원점
1. Material Girl
1. 모델 마무리하기

> **<i class="fa fa-exclamation-triangle" aria-hidden="true"></i> 주의:** 이 튜토리얼은 XML 코드 작성법을 숙지하고 있다고 가정한다.

## 목표
* Rviz에서 볼 수 있는, 로봇의 비주얼 모델 만드는 법 학습하기

이 튜토리얼에서는, R2D2와 비슷해보이는 로봇의 비주얼 모델을 만들 것이다. 이후 튜토리얼에서는, 모델을 표현하고, 물리적 특성을 추가하고, xacro를 이용하여 더 깔끔한 코드를 작성하는 법을 학습 할 것이다. 하지만, 지금은 시각적 형상을 올바르게 만드는데 집중할 것이다.

진행하기 전에, joint_state_publisher 패키지가 설치되어 있는지 확인하자. 만약 urdf_tutorial 바이너리를 설치했다면, 이미 설치되어 있을 것이다. 아니라면, 이 패키지를 포함하는 설치를 업데이트 하자(확인을 위해 rosdep을 사용).

```bash
sudo apt install ros-humble-urdf*
```

이 튜토리얼에서 언급된 모든 로봇 모델들은(및 소스 파일) urdf_tutorial 패키지에서 찾을 수 있다.

## 한가지 모양
먼저, 우리는 한가지 간단한 모양에 대해 살펴볼 것이다. 아래 코드는 직접 작성할 수 있을 만큼 간단한 urdf이다. [[Source:01-myfirst.urdf]](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/01-myfirst.urdf)

```xml 
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

XML을 번역해보면, '이것은 myfirst라는 이름의 로봇입니다. 단 하나의 링크(부품)만을 가지며, 그 비주얼 엘리먼트는 반지름 0.2[m], 높이 0.6[m]인 원기둥입니다.'가 된다. 이는 간단한 "hello world" 형태의 예제를 너무 많은 태그들이 둘로싸고 있는 것 처럼 보일 수 있다. 하지만 앞으로 더 복잡해 질 것이다.

모델을 테스트해보기 위해, display.launch.py 파일을 런치하자.

```bash
cd /opt/ros/humble/share/urdf_tutorial
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
```

실행하면, 다음과 같은 작업을 수행한다.

* 특정 모델을 로드하고, 파라미터도 저장
* sensor_msgs/msg/JointState를 publish하고 transform하는 노드를 실행(추후에 상세히 설명)
* 설정 파일이 적용된 Rviz 실행

런치 명령을 urdf_tutorial 패키지로부터 실행하고 있다는 것을 가정한다(즉, urdf 디렉토리는 현재 워킹 디렉토리의 바로 하위에 있는 디렉토리 이다). 그렇지 않다면, 01-myfirst.urdf의 상대 경로는 유효하지 않으며, 런쳐가 urdf를 파라미터로 로드할 때 에러가 발생할 것이다.

인자를 약간 수정하면, 워킹디렉토리에 상관없이 실행할 수 있다.

```bash
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf
```

만약 앞으로의 튜토리얼들에서 urdf_tutorial 패키지의 위치에서 실행하지 않는다면, 모든 예제 launch 명령을 위와 같이 변경해 주어야 한다. 

display.launch.py 실행 후, 다음과 같은 화면이 보일 것이다. 

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/myfirst.png)

**확인 사항**
* fixed 프레임은 격자의 중심이 위치한 transform 프레임이다. 여기서는 링크인 base_link에 의해 정의된 프레임이다.
* 비주얼 엘리먼트(원기둥)의 원점은 기본값으로 기하학적 중심에 위치한다. 또한 원기둥의 아래쪽 반은 격자 아래에 있다.

## 다양한 모양
이제 추가적인 모양/링크를 추가하는 법을 살펴보자. urdf에 더 많은 링크 엘리먼트들을 추가한다면, parser는 이것들을 어디에 위치시켜야 하는지 모를 것이다. 그러므로, 우리는 조인트를 추가해 주어야 한다. 조인트는 flexible 조인트와 unflexible 조인트가 있다. 먼저 inflexible(fixed) 조인트로 시작해 보자.[[Source: 02-multipleshapes.urdf]](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/02-multipleshapes.urdf)

```xml
<?xml version="1.0"?>
<robot name="multipleshapes">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
```

* 0.6m x 0.1m x 0.2m 박스를 정의한 방법 확인
* 조인트는 부모와 자식으로 정의된다. urdf는 근본적으로 하나의 루트 링크를 가진 트리 구조이다. 이는 다리는 위치가 base_link의 위치에 의존한다는 것을 의미한다.

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf
```

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/multipleshapes.png)

위 그림에서, 두 형상은 같은 원점을 공유하고 있기 때문에, 서로 겹쳐 보인다. 만약 이것들이 겹치지 않게 하려면, 더 많은 원점을 정의해야 한다.

## 원점
R2D2의 다리는 상반신에 한쪽 옆에 붙어 있다. 그래서, 이 위치가 바로 조인트의 원점을 정의할 위치이다. 또한 다리의 중간 부분이 아닌 윗부분이 붙어있다. 그래서, 다리의 원점에 오프셋을 줘야 한다. 그리고 위쪽을 향하도록 다리를 회전시켜야 주어야 한다.[[Source: 03-origins.urdf]](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/03-origins.urdf)

```xml
<?xml version="1.0"?>
<robot name="origins">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

</robot>
```

* 조인트의 원점을 살펴보자. 이것은 부모 프레임의 관점에서 정의되어 있다. 그래서 y방향으로(사진에서 왼쪽이나, 축 기준 오른쪽) -0.22미터, z방향으로(위) 0.25미터에 있다. 이는 자식 링크의 원점이 비주얼 원점 태그와 무관하게 오른쪽 위에 있다는 것을 의미한다. rpy속성은 명시하지 않았으므로, 기본적으로 자식 프레임은 부모와 동일한 방향이다.
* 이제 다리의 비주얼 원점을 살펴보자. 두개의 오프셋 xyz, rpy를 가지고 있다. 이는 원점 기준으로 비주얼 엘리먼트의 중심이 어디에 위치해야 하는지는 의미한다. 우리는 다리를 위쪽에 붙이고 싶기 때문에, 원점의 z 오프셋에 -0.3미터를 준다. 그리고 다리의 긴 쪽이 z축과 평행해야 하므로, y축 방향으로 pi/2만큼 회전시킨다.

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf
```

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/origins.png)

* 런치파일은 URDF를 기반으로한 모델의 각 링크에 대한 TF프레임을 만들 패키지를 실행한다. Rviz는 각 형상들이 어디에 표시되어야 하는지 알아내기 위해 이 정보를 사용한다. 
* 만약 TF 프레임이 URDF 링크에 대헤 존재하지 않으면, 흰색으로 원점에 배치된다.(참고. [관련 질문](http://answers.ros.org/question/207947/how-do-you-use-externally-defined-materials-in-a-urdfxacro-file/))

## Material Girl
재질 태그를 살펴보자. [[Source: 04-materials.urdf]](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/04-materials.urdf)

```xml
<?xml version="1.0"?>
<robot name="materials">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

</robot>
```

* 몸통은 이제 파란색이다. 우리는 red, green, blud, alpha 채널에 각각 0, 0, 0.8, 1 이라는 값으로 "blue"라는 새로운 재질을 정의했다. 모든 값들은 [0, 1]의 범위에 속한다. 이 재질은 base_link의 비주얼 엘리먼트에 의해 참조된다. 흰색 재질도 비슷하게 정의 된다.
* 비주얼 엘리먼트 안에서 재질 태그를 정의하는 것도 가능하며, 다른 링크에서 참조도 가능하다. 또한 그 태그를 재정의 하는 것도 가능핟. 
* 오브젝트에 이미지 파일을 구현하는 텍스쳐를 사용할 수 있다.

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf
```

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/materials.png)

## 모델 마무리하기
이제 발, 바퀴, 머리를 추가해 모델을 마무리 하자. 가장 주목해야 할 것은, 구(sphere)와 메쉬(mesh)이다. 또한, 추후 사용될 몇가지 다른 부분들도 추가 할 것이다.[[Source: 05-visual.urdf]](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/05-visual.urdf)

```xml
<?xml version="1.0"?>
<robot name="visual">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="right_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="right_base_joint" type="fixed">
    <parent link="right_leg"/>
    <child link="right_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="right_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_front_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="right_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_back_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

  <link name="left_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="left_base_joint" type="fixed">
    <parent link="left_leg"/>
    <child link="left_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="left_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_front_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_back_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <joint name="gripper_extension" type="fixed">
    <parent link="base_link"/>
    <child link="gripper_pole"/>
    <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
  </joint>

  <link name="gripper_pole">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.01"/>
      </geometry>
      <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
    </visual>
  </link>

  <joint name="left_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="left_gripper"/>
  </joint>

  <link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_tip_joint" type="fixed">
    <parent link="left_gripper"/>
    <child link="left_tip"/>
  </joint>

  <link name="left_tip">
    <visual>
      <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="right_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="right_gripper"/>
  </joint>

  <link name="right_gripper">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_tip_joint" type="fixed">
    <parent link="right_gripper"/>
    <child link="right_tip"/>
  </joint>

  <link name="right_tip">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  <joint name="head_swivel" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="box">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="tobox" type="fixed">
    <parent link="head"/>
    <child link="box"/>
    <origin xyz="0.1814 0 0.1414"/>
  </joint>
</robot>
```

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf
```

![](https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/images/visual.png)

구를 추가하는 방법은 아래와 같다.

```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.2"/>
    </geometry>
    <material name="white"/>
  </visual>
</link>
```

메쉬들은 PR2에서 차용했고, 각각의 파일들은 경로를 명시해야 한다. 우리는 여기서 package://NAME_OF_PACKAGE/path 표기법을 사용해야 한다. 이 튜토리얼에서의 메쉬들은 urdf_tutorial 패키지 안에 meshes라는 폴더에 위치한다.

```xml
<link name="left_gripper">
  <visual>
    <origin rpy="0.0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
    </geometry>
  </visual>
</link>
```

* 메쉬는 다양한 포멧으로 적용될 수 있다. STL이 자주 쓰이고, color/material을 명시할 필요 없이 자체 색 데이터를 가지는 DAE 또한 지원한다. 대게 여러개의 파일로 이루어져 있다. 이 메쉬들은 meshes 폴더의 .tif 파일을 참조하기도 한다.
* 메쉬는 스케일링 파라미터나 바운딩 박스 사이즈로 스케일링 할 수 있다.
* 메쉬는 외부 패키지에서도 참조 가능하다.

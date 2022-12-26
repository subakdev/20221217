# [Xacro로 코드 정리하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
1. 목표
1. Xacro 사용하기
1. 상수
1. 수학
1. 매크로
1. 실사용 예

## 목표
* Xacro를 사용하는 URDF파일에서 코드량 줄이는 트릭 학습하기.

지금까지의 과정에서는, 매우 간단한 로봇 description을 파싱하기 위해 필요한 수학들 푸느라 피곤했을지도 모르겠다. 다행스럽게도, [xacro](https://index.ros.org/p/xacro) 패키지를 사용하면 이 과정을 더 간단하게 만들 수 있다. 아래는 그 유용한 3개의 기능들이다.

* 상수
* 간단한 수학
* 매크로

이번 튜토리얼에서는 URDF 파일의 전체 사이즈를 줄여주고, 가독성 및 유지관리에 도움을 주는 숏컷들을 살펴볼 것이다.

## Xacro 사용하기
이름에서 유추할수 있듯이, [xacro](https://index.ros.org/p/xacro)는 XML의 매크로 언어이다. xacro 프로그램은 모든 매크로를 실행하고 결과물을 출력한다. 일반적인 사용법은 다음과 같다:

```bash
xacro model.xacro > model.urdf
```

launch file에서 urdf를 자동으로 생성하는 것도 가능하다. 이는 디스크 공간을 차지하지 않고, 문서를 항상 최신으로 유지할 수 있기 때문에 편리하다. 그러나 urdf를 생성하기 위해 시간이 걸리므로, 런치파일을 시작할 때 시간이 더 오래 걸리게 된다.

```python
from launch import LaunchDescription
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    path_to_urdf = get_package_share_path('urdf_tutorial') / 'urdf' / '08-macroed.urdf.xacro'
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf)]), value_type=str
            )
        }]
    ),

    return LaunchDescription(
        robot_state_publisher_node
    )
```

파일을 적절히 파싱하기 위해서는 URDF파일의 최상단에 반드시 네임스페이스를 명시해야 한다. 예를 들어, 유효한 xacro 파일의 처음 두 줄은 다음과 같다.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
```

## 상수
R2D2의 base_link를 살펴보자.

```xml
<?xml version="1.0" ?>
<robot name="firefighter">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

어떤 정보는 약간의 중복이 있다. 원기둥의 반지름과 높이를 두 번 정의하고 있다. 더 문제인 것은, 수정할 때 두 곳 다 수정해야 한다는 것이다.

다행히도, xacro는 상수역할을 하는 property를 명시하게 해준다. 코드 상단을 다음과 같이 바꿔쓸 수 있다.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">

<xacro:property name="width" value="0.2" />
<xacro:property name="bodylen" value="0.6" />
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
    <visual>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
        <material name="blue"/>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
    </collision>
</link>

</robot>
```

* 처음 두 라인은 두개의 값을 명시하고 있다. 이런 문장은 사용되기 전, 후에 어느 레벨 또는 위치에서든 정의 가능하다(XML이 유효하다고 가정하면). 일반적으로 상단에 정의 된다.
* geometry 요소에서 실제 반지름 값을 명시하는 것 대신, 달러 기호와 중괄호를 사용해서 값을 명시하고 있다.
* 이 코드는 처음 보았던 코드와 같은 코드를 생성할 것이다.

'${}' 구조에 있는 값은 '${}'를 대체하기 위해 사용된다. 이는 attribute안에 다른 text와 결합해서 사용도 가능하다는 것을 의미한다.

```xml
<xacro:property name=”robotname” value=”marvin” />
<link name=”${robotname}s_leg” />
```

위 구문은 아래 구문을 생성한다.

```xml
<link name="marvins_leg" />
```

그러나, ${}안의 내용이 property일 필요는 없다. 다음 섹션에서 이어서 설명한다.

## Math
${}구조에서 사직연산, 음수표현, 괄호를 사용해 복잡한 표현을 구성할 수 있다. 예를 들면,

```xml
<cylinder radius="${wheeldiam/2}" length="0.1"/>
<origin xyz="${reflect*(width+.02)} 0 0.25" />
```

sin, cos과 같은 기본 수학연산 이상의 것도 사용 가능하다.

## Macro
xacro 패키지에서 가장 유요한 부분은 이 섹션이다.

### Simple Macro
간단하고 무의미한 아래의 매크로를 모자

```xml
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
<xacro:default_origin />
```

(위는 원점을 명시하지 않았을 때의 기본값과 같기 때문에 무의미하다.) 위 코드는 아래를 생성할 것이다.

```xml
<origin rpy="0 0 0" xyz="0 0 0"/>
```

* name은 필수 요소는 아니지만 사용을 위해 필요하다.
* 모든 <xacro:$NAME /> 인스턴스는 xacro:macro 태그 내용으로 대체될 수 있다.
* xacro를 사용한 XML과 생성된 XML이 정확히 같지 않은 것처럼 보이지만(두 attribute의 순서가 반대임), 동일한 XML이다.
* 만약 name을 명시한 xacro가 존재하기 않을 경우, 확장되지도 않고, 에러를 생성하지도 않는다.

### 파라미터화된 매크로
xacro가 매번 정확히 같은 텍스트를 생성하지 않도록, 파라미터화 하는 것이 가능하다. 수학적인 기능과 함께 사용될 때, 훨씬 더 강력하다.

먼저. R2D2에서 사용된 간단한 macro 예제를 보자

```xml
<xacro:macro name="default_inertial" params="mass">
    <inertial>
            <mass value="${mass}" />
            <inertia ixx="1e-3" ixy="0.0" ixz="0.0"
                 iyy="1e-3" iyz="0.0"
                 izz="1e-3" />
    </inertial>
</xacro:macro>
```

위 코드는 아래와 같이 사용될 수 있다.
```xml
<xacro:default_inertial mass="10"/>
```

파라미터들은 property와 같은 역할을 하고, 문장에서 사용될 수 있다.

또한, 전체 블럭을 파라미터로 사용하는 것도 가능하다.

```xml
<xacro:macro name="blue_shape" params="name *shape">
    <link name="${name}">
        <visual>
            <geometry>
                <xacro:insert_block name="shape" />
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <xacro:insert_block name="shape" />
            </geometry>
        </collision>
    </link>
</xacro:macro>

<xacro:blue_shape name="base_link">
    <cylinder radius=".42" length=".01" />
</xacro:blue_shape>
```

* 블럭 파라미터를 명시하기 위해선, 파라미터 이름 앞에 별표를 붙여야 한다.
* 블럭은 insert_block 명령으로 삽입될 수 있다.
* 원하는 만큼 몇번이든 삽입 가능하다.

## 실사용 예
xacro 언어는 유연하다. 이제 위에서 봤던 관성적인 기본 매크로 뿐만 아니라, [R2D2 모델](https://github.com/ros/urdf_tutorial/blob/master/urdf/08-macroed.urdf.xacro)에 사용된 여러가지 강력한 기능을 알아보자.

xacro 파일로부터 생성된 모델을 보기위 아래 명령어를 실행하자

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro
```

(이제까지 실행해왔던 런치파일은 xacro 명령어가 있지만, 확장될 만한 것이 없이 때문에 큰 의미는 없었다.)

### R2D2 다리를 매크로로 만들기
종종, 서로 다른 위치에서 비슷한 여러 오브젝트를 만들 때가 있다. R2의 두 다리를 만들때 처럼, 작성해야할 코드량을 줄이기 위해 간단한 매크로와 수학을 활용할 수 있다.

```xml
<xacro:macro name="leg" params="prefix reflect">
    <link name="${prefix}_leg">
        <visual>
            <geometry>
                <box size="${leglen} 0.1 0.2"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
            <material name="white"/>
        </visual>
        <collision>
            <geometry>
                <box size="${leglen} 0.1 0.2"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
        </collision>
        <xacro:default_inertial mass="10"/>
    </link>

    <joint name="base_to_${prefix}_leg" type="fixed">
        <parent link="base_link"/>
        <child link="${prefix}_leg"/>
        <origin xyz="0 ${reflect*(width+.02)} 0.25" />
    </joint>
    <!-- A bunch of stuff cut -->
</xacro:macro>
<xacro:leg prefix="right" reflect="1" />
<xacro:leg prefix="left" reflect="-1" />
```

* 트릭 1 : 비슷한 이름의 오브젝트를 만들기 위해 접두사(prefix) 사용.
* 트릭 2 : 조인트의 원점을 계산하기 위해 수학을 사용. 로봇의 사이즈를 변경하는 경우, 조인트 오프셋을 계산하는 수학과 함께 property를 변경하는 것이 많은 문제를 예방할 수 있다.
* 트릭 3 : reflect 파라미터 사용해서 1 또는 -1로 셋팅. base_to_${prefix}_leg 원점에 있는 몸통의 양 쪽에 다리를 위치시키기 위해 reflect 파라미터를 사용하는 것을 확인해보자. 

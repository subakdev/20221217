# substitutions 사용하기
1. 목표
2. 배경지식
3. 사전준비
4. 실습
5. 요약

## 목표
* ROS2 launch 파일 내에서 substitutions에 대해서 학습

## 배경지식
* launch 파일의 역할
  * nodes 구동
  * services 구동
  * processes 실행(외부 프로그램 실행)
* argument(인자)가 필요할 수도 있다!
* Quiz : argument를 사용하면 좋은 점은?
* substitutions
  * 인자로 사용하는 경우
    * 유연성 제공
    * launch 파일 재사용성
  * 변수처럼 사용
    * launch 파일이 실행되는 시점에만 수행(환경 변수)
    * launch configuration
    * 환경 변수
    * 임의의 Python 코드
* 이 튜터리얼에서는 ROS2 launch 파일 내에서 substitutions의 예제로 배워보자.

## 사전준비
* turtlesim package 기본 이해
* package 만드는 방법 이해
* 새 터미널 열어서 source 명령 실행

## substitutions 사용하기

### 1. package 생성 및 설정
* launch_tutorial package 생성하기 (build_type은 ament_python)
```
ros2 pkg create launch_tutorial --build-type ament_python
```

* 해당 package 내부에 launch 디렉토리 생성
```
mkdir launch_tutorial/launch
```

* 마지막으로 setup.py에 변경 내용 추가하기
```python
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```

### 2. Parent launch 파일
* launch 파일 생성
  * 다른 launch 파일에 인자를 전달 
* launch_tutorial package 내부에 launch 디렉토리
  * example_main.launch.py 파일 생성
```python
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```


### 3. substitutions 예제 launch 파일
* 동일한 디렉토리에 example_substitutions.launch.py 파일 생성
```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle,
        change_background_r,
        TimerAction(
            period=2.0,
            actions=[change_background_r_conditioned],
        )
    ])
```

* example_substitutions.launch.py 파일 내에 launch 설정
  * turtlesim_ns
  * use_provided_red
  * new_background_r

* launch 인자(arguments)의 값을 저장하고 필요한 actions에 이를 전달하는 목적으로 사용
* LaunchConfiguration substitutions의 역할
  * launch 인자의 값에 접근 가능
* DeclareLaunchArgument는 launch 인자를 정의하는데 사용(launch 파일이나 console로부터 수신)
```python
turtlesim_ns = LaunchConfiguration('turtlesim_ns')
use_provided_red = LaunchConfiguration('use_provided_red')
new_background_r = LaunchConfiguration('new_background_r')

turtlesim_ns_launch_arg = DeclareLaunchArgument(
    'turtlesim_ns',
    default_value='turtlesim1'
)
use_provided_red_launch_arg = DeclareLaunchArgument(
    'use_provided_red',
    default_value='False'
)
new_background_r_launch_arg = DeclareLaunchArgument(
    'new_background_r',
    default_value='200'
)
```

### 4. package 빌드하기
* workspace 디렉토리로 가서 package 빌드하는 명령 수행
```
colcon build
```

## launching 예제

## launch 인자(argument) 수정하기

## 문서

## 요약

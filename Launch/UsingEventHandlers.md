# [event handlers 사용하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
1. 목표
2. 배경지식
3. 사전준비
4. event handlers 사용하기
   1. event handlers 예제 launch 파일
5. package 빌드하기
6. launching 예제
7. 문서
8. 요약

## 목표
* ROS2 launch 파일 내에서 event handler
## 배경지식
* ROS2에서 launch는 시스템이다. 
* 이 시스템은 user-defined processes를 실행하고 관리한다.
* 이 시스템은 launch된 processes의 상태를 모니터링하고 processes의 상태에서 변화에 대해서 리포팅하고 반응하는 책임을 수행한다.
* 이런 변경을 events이고 launch 시스템에서 event handler를 등록함으로써 처리할 수 있다. 
* event handler는 특정 event에 대해서 등록할 수 있어서 processes의 상태를 모니터링하는데 유용하다.
* 추가로 복잡한 원칙들을 적용하는 것도 가능하여 launch 파일을 동적으로 수정하는 것도 가능하다.

* 이 튜터리얼은 ROS2 launch 파일 내에서 event handlers의 예제를 보여준다. 

## 사전준비
* 이 튜터리얼에서는 turtlesim package를 사용한다.
* 새로운 package를 생성하고 ament_python build type을 사용할 수 있어야 한다. 
* 새로 생성할 package 이름은 launch_tutorial이다.

* 이 튜터리얼은 [launch 파일에서 substitutions 이용하기]()를 확장한다.

## event handlers 사용하기
### 1. event handlers 예제 launch 파일
* launch_tutorial package의 launch 디렉토리 내에 example_event_handlers.launch.py 라는 새로운 파일을 생성한다. 
```python
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


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
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
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
            FindExecutable(name='ros2'),
            ' param set ',
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
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=turtlesim_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
```

* 
## 요약
* 
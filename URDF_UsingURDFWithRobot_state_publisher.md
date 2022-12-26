# [URDF에서 robot_state_publisher 사용하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
1. 목표
1. 배경
1. 사전 준비
1. 실습
1. 요약

## 목표
* URDF로 모델링된 걸어다니는 로봇 시뮬레이션하고, Rviz로 확인하기

이번 튜토리얼은 걸어다니는 로봇을 모델링하고, tf2 메세지로 상태를 publish하고, Rviz 상에서 시뮬레이션하는 것을 다룰 것이다. 첫번째로는, 로봇 조립을 설명하는 URDF 모델을 만들 것이다. 그 다음으로, 움직임을 시뮬레이션하고 JointState 및 transform을 publish하는 노드를 만들 것이다. 우리는 /tf2/ 토픽으로 전체 로봇 상태를 publish하기 위해 robot_state_publisher를 사용할 것이다.

![](https://docs.ros.org/en/humble/_images/r2d2_rviz_demo.gif)

## 사전 준비
* [rviz2](https://index.ros.org/p/rviz2/)

언제나 새로운 터미널을 실행할때마다 ROS2 환경을 source하는 것을 잊지말자.

## 실습

### 1. 패키지 생성하기
```bash
mkdir -p ~/second_ros2_ws/src  # change as needed
cd ~/second_ros2_ws/src
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy
cd urdf_tutorial_r2d2
```

이제 urdf_tutorial_r2d2 폴더를 봐야 한다. 거기에 몇가지 변화를 줄 것이다.

### 2. URDF파일 생성하기
몇가지 에셋을 저장할 디렉토리를 만들자.

```bash
mkdir -p urdf
```

[URDF 파일](https://docs.ros.org/en/humble/_downloads/872802005223ffdb75b1ab7b25ad445b/r2d2.urdf.xml)을 다운로드하고 ~/second_ros2_ws/src/urdf_tutorial_r2d2/urdf/r2d2.urdf.xml 위치에 저장하자.
[Rviz 설정 파일](https://docs.ros.org/en/humble/_downloads/96d68aef72c4f27f32af5961ef48c475/r2d2.rviz)을 다운로드하고, ~/second_ros2_ws/src/urdf_tutorial_r2d2/urdf/r2d2.rviz 위치에 저장하자.

### 3. 상태 publish하기
이제 로봇이 어떤 상태인지 명시하기 위한 방법이 필요하다. 이를 위해 3개의 조인트 모두와 전체 odometry를 명시해야 한다.

~/second_ros2_ws/src/urdf_tutorial_r2d2/urdf_tutorial_r2d2/state_publisher.py 라는 이름으로 파일을 생성하고, 아래 내용을 입력하자.

```python
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['swivel', 'tilt', 'periscope']
                joint_state.position = [swivel, tilt, height]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
```

### 4. 런치파일 만들기
~/second_ros2_ws/src/urdf_tutorial_r2d2/launch 폴더를 새로 생성하자. 그리고 ~/second_ros2_ws/src/urdf_tutorial_r2d2/launch/demo.launch.py 파일을 생성하여 아래 내용을 집어 넣자.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'r2d2.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('urdf_tutorial_r2d2'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='urdf_tutorial_r2d2',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])
```

### 5. setup.py 파일 수정
colcon 빌드 툴에 파이썬 패키지를 설치할 방법을 전달해야 한다. ~/second_ros2_ws/src/urdf_tutorial_r2d2/setup.py 파일을 아래와 같이 수정하자

* import 문 추가

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
```

* data_files안에 아래 두 줄 추가

```python
data_files=[
  ...
  (os.path.join('share', package_name), glob('launch/*.py')),
  (os.path.join('share', package_name), glob('urdf/*'))
],
```

* 콘솔에서 state_publisher 실행을 위해 entry_points 테이블 수정

```python
'console_scripts': [
    'state_publisher = urdf_tutorial_r2d2.state_publisher:main'
],
```

setup.py를 저장한다.

### 6. 패키지 설치

```bash
cd ~/second_ros2_ws
colcon build --symlink-install --packages-select urdf_tutorial_r2d2
source install/setup.bash
```

### 7. 결과 확인
패키지를 실행하자

```bash
ros2 launch urdf_tutorial_r2d2 demo.launch.py
```

새로운 터미널을 열고 Rviz를 실행하자

```bash
rviz2 -d ~/second_ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
```

Rviz 사용법은 [유저 가이드](http://wiki.ros.org/rviz/UserGuide)를 참고하자.

## 요약
이제까지 JointState publisher 노드를 생성했고, robot_state_publisher와 결합하여 걸어다니는 로봇을 시뮬레이션 했다. 이 예제에서 사용된 코드는 [여기](https://github.com/benbongalon/ros2-migration/tree/master/urdf_tutorial)서 찾을 수 있다.

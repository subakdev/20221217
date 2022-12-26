# [data를 기록하고 플레이하기](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. 설정
   2. topic 정하기
   3. ros2 bag record
   4. ros2 bag info
   5. ros2 bag play
5. 요약

## 목표
* 어떤 topic으로 publish되는 data를 기록하여 재생하기

## 배경지식
* ros2 bag은 topics에서 publish되는 data를 기록하는 command line 도구이다.
* 이 data를 누적하여 DB에 저장한다.
* 이 data를 재생하여 테스트나 실험결과 확인
* 저장한 topics으로 작업 내용을 공유 가능
## 사전준비
* ros2 bag 설치
* 만약 설치가 안되었다면 아래 명령으로 설치
```bash
sudo apt-get install ros-humble-ros2bag \
                     ros-humble-rosbag2-storage-default-plugins
```
* nodes, topics 에 대한 이해
* turtlesim package 사용해서 실습 진행
* 새 터미널 열어서 ROS2 환경 source 하기
## 실습
###   1. 설정
* turtlesim 시스템에 키보드 입력을 저장하여 나중에 replay할 예정이다. 
* /turtlesim과 /teleop_turtle nodes를 시작시키자.
* 새 터미널 열어서 실행하기
```
ros2 run turtlesim turtlesim_node
```
* 또 새 터미널 열어서 실행하기
```
ros2 run turtlesim turtle_teleop_key
```
* 새 디렉토리에 저장시키는 것이 좋은 습관이다.
```
mkdir bag_files
cd bag_files
```
###   2. topic 정하기
* ros2 bag 명령은 topic으로 publish되는 data만 저장할 수 있다.
* 현재 시스템에서 topics 목록을 보려면 새 터미널을 열고 다음 명령을 수행한다.
```
ros2 topic list
```
* 결과 :
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
* topics 튜터리얼에서 /turtle_teleop node가 /turtle1/cmd_vel topic으로 명령을 publish하여 turtlesim에서 turtle이 움직이게 한다든 것을 배웠다.
* /turtle1/cmd_vel 이 publish하는 data를 보려면 다음 명령을 수행한다.
```
ros2 topic echo /turtle1/cmd_vel
```
* 처음에는 아무런 data도 보이지 않는다. 왜냐하면 teleop이 publish하는 data가 없기 때문이다.
* teleop를 실행시켰던 터미널로 돌아와서 활성화시키기 위해서 이를 선택한다. 화살표키를 이용하여 turtle을 이동시키면 ros2 topic echo 명령을 실행해서 publish되는 data를 볼 수 있다. 

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

###   3. ros2 bag record
* 특정 topic으로 publish되는 data를 저장하기 위한 명령 문법은 다음과 같다.
```
ros2 bag record <topic_name>
```
* 선택한 topic에 대해서 이 명령을 실행하기 전에,  새 터미널을 열고 이전에 생성한 bag_files 디렉토리로 이동하자. 왜냐하면 rosbag 파일은 명령을 실행하는 디렉토리 내에 저장된다.
* 명령 실행하기
```
ros2 bag record /turtle1/cmd_vel
```
* 해당 터미널에서 다음과 같은 메시지를 볼 수 있다.
```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```
* 이제 ros2 bag이 /turtle1/cmd_vel topic으로 publish되는 data를 저장한다. 
* teleop 터미널로 돌아와서 turtle을 움직인다. 움직임 자체가 중요한 것이 아니라 나중에 replay할때 움직이는 패턴을 기억할 수 있게 움직이는 패턴을 생각하고 움직이자.

![](https://docs.ros.org/en/humble/_images/record.png)

* Ctrl+c를 눌러 record를 중지한다.

* data는 rosbag2_year_month_day-hour_minute_second 이름 패턴을 가지는 bag 파일에 계속 쌓인다.
### 3.1 여러 topics 저장하기
* ros2 bag 파일 이름 변경 및 여러 topics을 저장할 수 있다.
* 다음 명령을 실행하자.
```
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```
* -o 옵션은 bag 파일 이름을 정할 수 있다. 이 경우 subset 이후에 문자열이 바로 파일 이름이다.

* 1개 이상의 topics에 대해서 저장하기 위해서 space로 각 topic을 구분해서 작성하면 된다.

* 2개 topics이 저장되고 있다면 다음과 같이 메시지를 출력한다.
```
[INFO] [rosbag2_storage]: Opened database 'subset'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```
* turtle을 이동시키고 멈추려면 Ctrl+C 키를 누른다.
* Note
  * topics을 추가하는 또 다른 옵션은 -a가 있다. 이렇게 하면 모든 topics을 저장하게 된다.

###   4. ros2 bag info
* 저장에 관련된 상세정보를 보려면 다음과 같이 실행한다.
```
ros2 bag info <bag_file_name>
```
* subset bag 파일에서 이 명령을 수행하면 해당 파일에 대한 정보 목록을 반환한다.
```
ros2 bag info subset
```
```
Files:             subset.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                 Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```

* 개별 message를 보려면 sqlite3 DB를 열어야 하는데 이것은 ROS2 범위를 벗어나는 것이라 더 다루지는 않는다.

###   5. ros2 bag play
* bag file을 replay하기 전에, teleop가 실행 중인 터미널에서 Ctrl+C 입력한다. bag 파일이 실행되는 것을 보려면 turtlesim 윈도우가 보이는 상태를 유지하고 있어야 한다.

* 다음 명령을 실행한다.
```
ros2 bag play subset
```

* 터미널은 다음 메시지를 반환한다.
```
[INFO] [rosbag2_storage]: Opened database 'subset'.
```

* turtle은 저장했던 path와 동일한 path를 따라간다. (100% 정확하지는 않을 수 있음. turtlesim이 시간 타이밍에 민감할 수 있어서)

![](https://docs.ros.org/en/humble/_images/playback.png)

* subset 파일은 /turtle1/pose topic을 저장하기 때문에 ros bag play 명령은 turtlesim을 실행시키고 있는 한 종료되지 않는다.(심지어 움직이지 않고 있더라도 종료된 것은 아니다.)

* /turtlesim node가 계속 활성화되어 있기 때문이다. /turtle1/pose topic으로 일정 주기로 data를 publish 한다.
* 위에서 봤던 ros2 bag info 예제에서 /turtle1/cmd_vel topic의 Count 정보는 9였다. 이것으로 recording하는 동안 화살표키를 몇 번 눌렀는지 알 수 있다.

* /turtle1/pose는 Count 값이 3000이 넘었다. recording하는 동안 해당 topic으로 3000번 이상이 publish되었다는 것이다.

* 얼마나 자주 position data가 publish되었는지 알려면 다음 명령을 실행한다.
```
ros2 topic hz /turtle1/pose
```

## 요약
* ROS2 시스템에서 topic을 지나가는 data를 저장하는 명령이 바로 ros2 bag 명령이다.
* 다름 사람들과 작업을 공유하거나 실험을 다시 돌려볼때 이 도구를 이용한다.

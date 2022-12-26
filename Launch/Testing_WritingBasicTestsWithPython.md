# [Testing - Python으로 기본 테스트 작성하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html)

* 시작지점 : 이미 기본 ament_python package 설정이 되었다고 가정한다. 여기에 몇 가지 테스트를 추가해보자.
* 만약에 ament_cmake_python을 사용하고 있다면 ament_cmake_python 문서를 보고 tests를 만드는 방법을 참고하자. colcon으로 test contents와 호출하는 것은 동일하다.

1. Package 설정
   1. setup.py
   2. Test 파일과 폴더
   3. 예제 package layout
2. Test Contents
3. 특수 명령


## Package 설정
### 1. setup.py
* setup.py는 반드시 pytest에 test 의존성을 추가해야한다. :
```python
tests_require=['pytest'],
```

### 2. Test 파일과 폴더
* test 코드의 위치는 package 디렉토리 아래 tests 디렉토리이다.
* tests를 포함하는 파일의 형식은 test_FOO.py와 같은 패턴을 가진다. 여기서 FOO 부분을 원하는 이름으로 변경할 수 있다.

### 3. 예제 package layout
```
awesome_ros_package/
  awesome_ros_package/
      __init__.py
      fozzie.py
  package.xml
  setup.cfg
  setup.py
  tests/
      test_init.py
      test_copyright.py
      test_fozzie.py
```
## Test Contents
* tests를 heart의 content에 작성할 수 있다.
* [pytest 관련 문서](https://docs.pytest.org/en/7.2.x/) 참고
* 함수의 형식은 test_ 형태를 가진다.
```python
def test_math():
    assert 2 + 2 == 5   # This should fail for most mathematical systems
```

## 특수 명령
* [표준 colcon test 명령](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html) 에서 arguments를 지정할 수도 있다. pytest에 --pytest-args flag를 사용한다.
* 아래 예제는 실행한 test 함수의 이름을 지정할 수 있다.
```
colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function
```

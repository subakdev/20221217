#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node  // node 상속
{
public:
  MinimalParam()            // 생성자
  : Node("minimal_param_node")  // node 이름
  {
    this->declare_parameter("my_parameter", "world");  // my_parameter라는 prameter의 값을 "world"로 선언

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this)); // 1초 timer 생성
  }

  void timer_callback()  // 1초마다 실행되는 callback 정의
  {
    std::string my_param =
      this->get_parameter("my_parameter").get_parameter_value().get<std::string>();  // my_parameter라는 parameter의 값을 가져오기

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());  // Hello 뒤에 my_parameter 값을 출력

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")}; // 새 my_parameter의 "world" 값으로 생성
    this->set_parameters(all_new_parameters);   // my_parameter라는 parameter를 다시 설정
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
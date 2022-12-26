#include <memory>

#include "rclcpp/rclcpp.hpp"        //ROS client libraries
#include "std_msgs/msg/string.hpp" //subscription message type
using std::placeholders::_1; // std::bind

class MinimalSubscriber : public rclcpp::Node  // Node를 상속
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(         //subscription 생성
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));    // msg를 수신하면 처리하는 callback 선언
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const   // 실제 subscription이 발생하면 처리하는 부분( 인자로 )
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());   // 수신한 string을 출력
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());  //node를 인자로 instance 생성하여 
  rclcpp::shutdown();
  return 0;
}
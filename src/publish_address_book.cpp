#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp" // ROS client libraries
#include "more_interfaces/msg/address_book.hpp" // msg  type

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node  // Node 상속
{
public:
  AddressBookPublisher()  //생성자
  : Node("address_book_publisher") // Node 이름
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10); // publisher 생성

    auto publish_msg = [this]() -> void {                   // publisher callback 정의
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";            // msg 채우기
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);                // 실제 publish
      };
    timer_ = this->create_wall_timer(1s, publish_msg); // 1초마다 publish하는 timer 생성
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());  //  node 실행
  rclcpp::shutdown();

  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "my_msgs/msg/pos.hpp"

#include "maze_solver/search/trajectory_creator.hpp"

using std::placeholders::_1;

class MyPublisher : public rclcpp::Node {
public:
  MyPublisher() : Node("my_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    publisher2_ = this->create_publisher<my_msgs::msg::Pos>("pos_topic", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&MyPublisher::publishMessage, this));
  }

  typedef std::shared_ptr<my_msgs::msg::Pos> PosPtr;
  // typedef message_filters::Subscriber<my_msgs::msg::Pos> PosSub;

  // std::vector<PosPtr> m_subscribers;

private:
  void publishMessage() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    auto msg = my_msgs::msg::Pos();
    auto header = this->now();
    msg.x = 1;
    msg.y = 2;
    publisher2_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<my_msgs::msg::Pos>::SharedPtr publisher2_;

  // auto publisher = node->create_publisher<MyCustomMessage>("my_topic", 10);

  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyPublisher>();

  auto subscription = node->create_subscription<my_msgs::msg::Pos>(
      "pos_topic", 10, [](const my_msgs::msg::Pos::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("my_subscriber_node"), //
                    "Received message:%f, %f", msg->x, msg->y);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"                              // Standard string message
#include "tutorial_rclcpp_pub_sub/msg/tutorial_client.hpp"      // Custom message

using namespace std::chrono_literals;

class MinimalRclcppNode : public rclcpp::Node
{
public:
    MinimalRclcppNode()
    : Node("rclcpp_publisher"), count_(0)
    {
        // Create a publisher
        publisher_ = this->create_publisher<tutorial_rclcpp_pub_sub::msg::TutorialClient>(      // Message type
            "topic",                            // Topic to publish
            10);

        // Create a timer loop
        timer_ = this->create_wall_timer(
            500ms,                                          // Timer interval
            std::bind(&MinimalRclcppNode::cb_timer_update,  // Callback function
            this));
    }

private:
    void cb_timer_update()
    {
        auto message = tutorial_rclcpp_pub_sub::msg::TutorialClient();                               // CHANGE
        message.name = "Foxy";
        message.age = 25;
        message.count = this->count_++;                                        // CHANGE
        //RCLCPP_INFO(this->get_logger(), "Publishing: Name:'%s' AGE:'%d' Count: %d", message.name.c_str(), message.age, message.count);    // CHANGE
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_rclcpp_pub_sub::msg::TutorialClient>::SharedPtr publisher_;      //Publisher    
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalRclcppNode>());
  rclcpp::shutdown();
  return 0;
}

#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"                              // Standard string message
#include "tutorial_rclcpp_pub_sub/msg/tutorial_client.hpp"      // Custom message
#include "tutorial_rclcpp_service/srv/tutorial_service_add.hpp" // Custom service

using namespace std::chrono_literals;

class MinimalRclcppNode : public rclcpp::Node
{
public:
    MinimalRclcppNode()
    : Node("rclcpp_service_client"), count_(0)
    {
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Create a client to handle service
        client_ = this->create_client<tutorial_rclcpp_service::srv::TutorialServiceAdd>(
            "add_two_ints", 
            rmw_qos_profile_services_default, 
            client_cb_group_);
        // Create a timer loop
        timer_ = this->create_wall_timer(
            500ms,                                          // Timer interval
            std::bind(&MinimalRclcppNode::cb_timer_update, this), // Callback function
            timer_cb_group_);
    }

private:
    void cb_timer_update()
    {
        if(client_->service_is_ready()) {
            auto request = std::make_shared<tutorial_rclcpp_service::srv::TutorialServiceAdd::Request>();
            request->a = 10;
            request->b = 55;
            auto client_future_ = client_->async_send_request(request);
            // Do this instead of rclcpp::spin_until_future_complete()
            std::future_status status = client_future_.wait_for(10s);  // timeout to guarantee a graceful finish
            if (status == std::future_status::ready) {
                std::shared_ptr<tutorial_rclcpp_service::srv::TutorialServiceAdd::Response> response = client_future_.get();
                // Do something with response 
                RCLCPP_INFO(this->get_logger(), "Received response: %d", response->sum);
            }
        }
    }
    
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Client<tutorial_rclcpp_service::srv::TutorialServiceAdd>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<MinimalRclcppNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

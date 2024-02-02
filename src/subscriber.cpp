#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

using std::placeholders::_1;

class JointSubscriber : public rclcpp::Node
{
public:
    JointSubscriber()
    : Node("joint_subscriber")
    {
        subscription_ = this->create_subscription<tachimawari_interfaces::msg::Joint>(
            "Joint", 10, std::bind(&JointSubscriber::topic_callback, this, _1));
    }
private:
    void topic_callback(const tachimawari_interfaces::msg::Joint & msg) const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Report " << msg.id << ":" << msg.position);
    }
    rclcpp::Subscription<tachimawari_interfaces::msg::Joint>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSubscriber>());
    rclcpp::shutdown();
}
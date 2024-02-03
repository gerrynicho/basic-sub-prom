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
        joint_one.Set(msg.position);
    }
    rclcpp::Subscription<tachimawari_interfaces::msg::Joint>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
    Exposer exposer{"127.0.0.1:7500"}; //localhost
    auto registry = std::make_shared<Registry>();
    auto& Joint = BuildGauge()
                    .Name("joint_data")
                    .Help("Joints that will be monitored")
                    .Register(*registry);
    auto& joint_one = Joint.Add({{"id", "one"}});
    auto& joint_two = Joint.Add({{"id", "two"}});
    exposer.RegisterCollectable(registry);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSubscriber>());
    rclcpp::shutdown();
}
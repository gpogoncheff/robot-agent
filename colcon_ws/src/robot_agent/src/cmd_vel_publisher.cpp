#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node {
    public:
        CmdVelPublisher() : Node("cmd_vel_publisher"), step_(0) {
            pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&CmdVelPublisher::on_timer, this)
            );
            RCLCPP_INFO(this->get_logger(), "cmd_vel publisher started");
        }

    private:
        void on_timer() {
            geometry_msgs::msg::TwistStamped msg;

            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "base_link";

            if (step_ < 30) {
                msg.twist.linear.x = 0.2;
                msg.twist.angular.z = 0.0;
            } else if (step_ < 60) {
                msg.twist.linear.x = 0.0;
                msg.twist.angular.z = 0.5;
            } else {
                msg.twist.linear.x = 0.0;
                msg.twist.angular.z = 0.0;
            }

            pub_->publish(msg);
            ++step_;
        }

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        int step_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
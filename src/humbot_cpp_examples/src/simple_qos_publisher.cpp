#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimpleQoSPublisher : public rclcpp::Node{

    public:
        SimpleQoSPublisher() : Node("simple_qos_publisher"), qos_profile_pub_(10), counter_(0)
        {
            declare_parameter<std::string>("reliability", "system_default");
            declare_parameter<std::string>("duarability", "system_default");

            const auto reliability = get_parameter("reliability").as_string();
            const auto duarability = get_parameter("duarability").as_string();

            if(reliability == "best_effort"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                RCLCPP_INFO(get_logger(), "[Reliability]: Best Effort");
            }
            else if (reliability == "reliable"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                RCLCPP_INFO(get_logger(), "[Reliability]: Reliable");
            }
            else if(reliability == "system_default"){
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Reliability]: System Default");
            }
            else{
                RCLCPP_ERROR_STREAM(get_logger(), "Selected Reliability QoS: " << reliability << "doesn't exixts!");
                return;
            }

            if(duarability == "volatile"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                RCLCPP_INFO(get_logger(), "[Durability]: Volatile");
            }
            else if(duarability == "transient_local"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                RCLCPP_INFO(get_logger(), "[Durability]: Transient local");
            }
            else if(duarability == "system_default"){
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Durability]: System Default");
            }
            else{
                RCLCPP_ERROR_STREAM(get_logger(), "Selected Durability QoS: " << duarability << "doesn't exixts!");
                return;
            }

            pub_ = create_publisher<std_msgs::msg::String>("chatter", qos_profile_pub_);
            timer_ = create_wall_timer(1s, std::bind(&SimpleQoSPublisher::timerCallback, this));

            RCLCPP_INFO(this->get_logger(), "Publishing at 1 Hz");
        }

        void timerCallback(){
            counter_ += 1;
            auto message = std_msgs::msg::String();
            message.data = " Ishika is better than " + std::to_string(counter_);
            pub_->publish(message);

        }

    private: 
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::QoS qos_profile_pub_;
        
};

int main( int argc, char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleQoSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

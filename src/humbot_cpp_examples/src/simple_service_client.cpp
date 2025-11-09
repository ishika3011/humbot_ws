#include <rclcpp/rclcpp.hpp>
#include <humbot_msgs/srv/add_two_ints.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient : public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        client_ = create_client<humbot_msgs::srv::AddTwoInts>("add_two_ints");

        auto request = std::make_shared<humbot_msgs::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        while(!client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");

        }
        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));

        RCLCPP_INFO_STREAM(get_logger(), "Requesting sum of " << a << " and " << b);


    }
private:

    rclcpp::Client<humbot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<humbot_msgs::srv::AddTwoInts>::SharedFuture future) //accessing the future response of other thread
    {
        if(future.valid())
        {
            RCLCPP_INFO_STREAM(get_logger(), "Service Response: " << future.get()->sum);
            return; 
        }
        else{
            RCLCPP_ERROR(get_logger(), "Service failure");
        }
        
    }


};

int main(int argc,char* argv[]){

    rclcpp::init(argc, argv);
    if(argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments. Usage: simple_service_client <a> <b>");
        return 1;
    }

    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2])); // Example values for a and b
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
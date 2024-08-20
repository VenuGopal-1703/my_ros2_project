
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <chrono>  // Include this header for std::chrono

class SinusoidalMovement : public rclcpp::Node
{
public:
    SinusoidalMovement() : Node("turtlebot_movement"), amplitude_(2.0), frequency_(0.5), linear_speed_(0.2) 
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Use std::chrono::milliseconds
            std::bind(&SinusoidalMovement::move_in_sinusoidal_pattern, this));
        time_ = 0.0;
        //log info  message that node started succesfully
        RCLCPP_INFO(this->get_logger(), "Node started succesfully");
    }

private:
    void move_in_sinusoidal_pattern()
    {
        auto message = geometry_msgs::msg::Twist();
        
        // Linear velocity along x-axis
        message.linear.x = linear_speed_ ;

        // Angular velocity to create sinusoidal movement along y-axis
        message.angular.z = amplitude_ * std::sin(2 * M_PI * frequency_ * time_);

        

        // Publish the velocity command
        publisher_->publish(message);

        // Increment time
        time_ += 0.5;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double amplitude_, frequency_, linear_speed_, time_;
   // double direction_; //declaration of direction 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); //ros2 intialisation 
    rclcpp::spin(std::make_shared<SinusoidalMovement>());
    rclcpp::shutdown();
    return 0;
}

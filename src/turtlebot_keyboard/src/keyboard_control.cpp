#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class KeyboardControlNode : public rclcpp::Node
{
public:
    KeyboardControlNode() : Node("keyboard_control_node")
    {
        // Initialize publisher with topic name and queue size
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Set up a timer to call publish_cmd() at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->publish_cmd(); }
        );

        // Log message indicating that the node has started successfully
        RCLCPP_INFO(this->get_logger(), "KeyboardControlNode has been started successfully.");
    }

private:
    // Function to publish the command message
    void publish_cmd()
    {
        geometry_msgs::msg::Twist msg;
        char c = get_key();  // Get key press from terminal
        
        // Debugging statement
        RCLCPP_INFO(this->get_logger(), "Key pressed: %c", c);
        
        switch (c)
        {
        case 'w': // Move forward
            msg.linear.x = 0.5;
            msg.linear.y = 0.0;
            break;
        case 's': // Move backward
            msg.linear.x = -0.5;
            msg.linear.y = 0.0;
            break;
        case 'a': // Move left
            msg.linear.x = 0.0;
            msg.linear.y = 0.5;
            break;
        case 'd': // Move right
            msg.linear.x = 0.0;
            msg.linear.y = -0.5;
            break;
        case 'q': // Turn left
            msg.angular.z = 0.5;
            break;
        case 'e': // Turn right
            msg.angular.z = -0.5;
            break;
        case 'x': // Stop
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.angular.z = 0.0;
            break;
        default:
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.angular.z = 0.0;
            break;
        }
        
        // Log the message being published
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%f,linear.y=%f ,angular.z = %f", msg.linear.x, msg.linear.y , msg.angular.z);
        publisher_->publish(msg);
    }

    // Function to get a single key press from the terminal
    char get_key()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);  // Get current terminal attributes
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Apply new settings
        ch = getchar();  // Read character from standard input
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old settings
        return ch;
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    auto node = std::make_shared<KeyboardControlNode>();  // Create node instance
    rclcpp::spin(node);  // Enter the spin loop to keep node active
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

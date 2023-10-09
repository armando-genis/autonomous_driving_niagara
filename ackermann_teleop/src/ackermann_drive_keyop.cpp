#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

#define MAX_SPEED 5.0
#define MAX_STEERING_ANGLE 0.7


class ackermann_drive_keyop : public rclcpp::Node
{
private:
    double speed_ = 0.0;
    double steering_angle_ = 0.0;

    // function prototypes
    void pub_callback();
    void print_instructions();
    char getch();
    void finalize();

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    ackermann_drive_keyop(/* args */);
    ~ackermann_drive_keyop();
    void key_loop();
};

ackermann_drive_keyop::ackermann_drive_keyop(/* args */) : Node("ackermann_drive_keyop_node")
{
    motor_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ackermann_drive_keyop::pub_callback, this));
    print_instructions();

    RCLCPP_INFO(this->get_logger(), "ackermann_drive_keyop_node initialized");

}

ackermann_drive_keyop::~ackermann_drive_keyop()
{
    finalize();
}

void ackermann_drive_keyop::print_instructions()
{
    RCLCPP_INFO(this->get_logger(), "Use arrow keys to control speed and steering. Space to brake, tab to align wheels.");
}



char ackermann_drive_keyop::getch()
{
    char buf = 0;
    struct termios old;
    old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");

    fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(0, &read_fds);  // FD 0 is stdin
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 0.1 seconds
    int num_ready = select(1, &read_fds, NULL, NULL, &timeout);
    if (num_ready > 0)
    {
        if (read(0, &buf, 1) < 0)
            perror("read()");
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return (buf);
}



void ackermann_drive_keyop::key_loop()
{
    char c;
    while (rclcpp::ok())
    {
        c = getch();   // call your non-blocking input function
        switch(c)
        {
            case '\x41':  // Up arrow
                speed_ += 0.05;
                // RCLCPP_INFO(this->get_logger(), "up array pressed");

                break;
            case '\x42':  // Down arrow
                speed_ -= 0.05;
                break;
            case '\x43':  // Right arrow
                steering_angle_ -= 0.1;
                break;
            case '\x44':  // Left arrow
                steering_angle_ += 0.1;
                break;
            case '\x20':  // Space
                speed_ = 0.0;
                break;
            case '\x09':  // Tab
                steering_angle_ = 0.0;
                break;
            default:
                break;
        }
        speed_ = std::clamp(speed_, -MAX_SPEED, MAX_SPEED);
        steering_angle_ = std::clamp(steering_angle_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
        // RCLCPP_INFO(this->get_logger(), "speed: %f, steering_angle: %f", speed_, steering_angle_);
    }
}

void ackermann_drive_keyop::pub_callback()
{
    auto ackermann_cmd_msg = ackermann_msgs::msg::AckermannDriveStamped();
    ackermann_cmd_msg.drive.speed = speed_;
    ackermann_cmd_msg.drive.steering_angle = steering_angle_;
    motor_pub_->publish(ackermann_cmd_msg);
    RCLCPP_INFO(this->get_logger(), "speed: %f, steering_angle: %f", speed_, steering_angle_);
}

void ackermann_drive_keyop::finalize()
{
    auto ackermann_cmd_msg = ackermann_msgs::msg::AckermannDriveStamped();
    ackermann_cmd_msg.header.stamp = this->now();  // Set the timestamp to the current time
    ackermann_cmd_msg.header.frame_id = "base_link";
    ackermann_cmd_msg.drive.speed = 0;
    ackermann_cmd_msg.drive.steering_angle = 0;
    motor_pub_->publish(ackermann_cmd_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ackermann_drive_keyop>();
    std::thread key_loop_thread(&ackermann_drive_keyop::key_loop, node);  // Create a thread for the key_loop method
    rclcpp::spin(node); 
    key_loop_thread.join();  
    rclcpp::shutdown();
    return 0;
}
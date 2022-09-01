#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "bfc_msgs/msg/coordination.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class coordination : public rclcpp::Node
{
public:
    coordination()
        : Node("coordination")
    {
        this->declare_parameter("robotNumber", 0);
        robotNumber = this->get_parameter("robotNumber").as_int();

        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "coordination_data_" + std::to_string(robotNumber), 10, std::bind(&coordination::subscribeData, this, _1));

        if (robotNumber == 1)
        {
            robot1Publisher_ = this->create_publisher<bfc_msgs::msg::Coordination>("robot_satu", 10);
        }

        if (robotNumber == 2)
        {
            robot2Publisher_ = this->create_publisher<bfc_msgs::msg::Coordination>("robot_dua", 10);
        }

        if (robotNumber == 3)
        {
            robot3Publisher_ = this->create_publisher<bfc_msgs::msg::Coordination>("robot_tiga", 10);
        }

        if (robotNumber == 4)
        {
            robot4Publisher_ = this->create_publisher<bfc_msgs::msg::Coordination>("robot_empat", 10);
        }

        if (robotNumber == 5)
        {
            robot5Publisher_ = this->create_publisher<bfc_msgs::msg::Coordination>("robot_lima", 10);
        }

        timer_ = this->create_wall_timer(
            35ms, std::bind(&coordination::publishData, this));
    }

private:
    void publishData()
    {
        if (robotNumber == 1)
        {
            auto message = bfc_msgs::msg::Coordination();
            message.robot_number = robot1Id;
            message.status = robot1Status;
            message.state = robot1State;
            message.grid_position = robot1GridPosition;
            message.found_ball = robot1FBall;
            message.distance_ball = robot1DBall;
            message.grid_ball = robot1GridBall;
            message.back_in = robot1BackIn;
            robot1Publisher_->publish(message);
        }

        if (robotNumber == 2)
        {
            auto message = bfc_msgs::msg::Coordination();
            message.robot_number = robot2Id;
            message.status = robot2Status;
            message.state = robot2State;
            message.grid_position = robot2GridPosition;
            message.found_ball = robot2FBall;
            message.distance_ball = robot2DBall;
            message.grid_ball = robot2GridBall;
            message.back_in = robot2BackIn;
            robot2Publisher_->publish(message);
        }

        if (robotNumber == 3)
        {
            auto message = bfc_msgs::msg::Coordination();
            message.robot_number = robot3Id;
            message.status = robot3Status;
            message.state = robot3State;
            message.grid_position = robot3GridPosition;
            message.found_ball = robot3FBall;
            message.distance_ball = robot3DBall;
            message.grid_ball = robot3GridBall;
            message.back_in = robot3BackIn;
            robot3Publisher_->publish(message);
        }

        if (robotNumber == 4)
        {
            auto message = bfc_msgs::msg::Coordination();
            message.robot_number = robot4Id;
            message.status = robot4Status;
            message.state = robot4State;
            message.grid_position = robot4GridPosition;
            message.found_ball = robot4FBall;
            message.distance_ball = robot4DBall;
            message.grid_ball = robot4GridBall;
            message.back_in = robot4BackIn;
            robot4Publisher_->publish(message);
        }

        if (robotNumber == 5)
        {
            auto message = bfc_msgs::msg::Coordination();
            message.robot_number = robot5Id;
            message.status = robot5Status;
            message.state = robot5State;
            message.grid_position = robot5GridPosition;
            message.found_ball = robot5FBall;
            message.distance_ball = robot5DBall;
            message.grid_ball = robot5GridBall;
            message.back_in = robot5BackIn;
            robot5Publisher_->publish(message);
        }
    }

    void subscribeData(const std_msgs::msg::Int16MultiArray::SharedPtr message)
    {
        if (robotNumber == 1)
        {
            robot1Id = message->data[0];
            robot1Status = message->data[1];
            robot1State = message->data[2];
            robot1GridPosition = message->data[3];
            robot1FBall = message->data[4];
            robot1DBall = message->data[5];
            robot1GridBall = message->data[6];
            robot1BackIn = message->data[7];
        }

        if (robotNumber == 2)
        {
            robot2Id = message->data[0];
            robot2Status = message->data[1];
            robot2State = message->data[2];
            robot2GridPosition = message->data[3];
            robot2FBall = message->data[4];
            robot2DBall = message->data[5];
            robot2GridBall = message->data[6];
            robot2BackIn = message->data[7];
        }

        if (robotNumber == 3)
        {
            robot3Id = message->data[0];
            robot3Status = message->data[1];
            robot3State = message->data[2];
            robot3GridPosition = message->data[3];
            robot3FBall = message->data[4];
            robot3DBall = message->data[5];
            robot3GridBall = message->data[6];
            robot3BackIn = message->data[7];
        }

        if (robotNumber == 4)
        {
            robot4Id = message->data[0];
            robot4Status = message->data[1];
            robot4State = message->data[2];
            robot4GridPosition = message->data[3];
            robot4FBall = message->data[4];
            robot4DBall = message->data[5];
            robot4GridBall = message->data[6];
            robot4BackIn = message->data[7];
        }

        if (robotNumber == 5)
        {
            robot5Id = message->data[0];
            robot5Status = message->data[1];
            robot5State = message->data[2];
            robot5GridPosition = message->data[3];
            robot5FBall = message->data[4];
            robot5DBall = message->data[5];
            robot5GridBall = message->data[6];
            robot5BackIn = message->data[7];
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robot1Publisher_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robot2Publisher_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robot3Publisher_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robot4Publisher_;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robot5Publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
    int robotNumber;
    int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall, robot1BackIn;
    int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall, robot2BackIn;
    int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall, robot3BackIn;
    int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall, robot4BackIn;
    int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall, robot5BackIn;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<coordination>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "rmf_hamal/hamal_task_dispatcher.hpp"

HamalTaskDispatcherNode::HamalTaskDispatcherNode() : Node("hamal_task_dispatcher")
{
    bid_response_sub_ = create_subscription<rmf_task_msgs::msg::BidResponse>(
        "/rmf_task/bid_response", 10,
        std::bind(&HamalTaskDispatcherNode::bid_response_callback, this, std::placeholders::_1));

    dispenser_request_sub_ = create_subscription<rmf_dispenser_msgs::msg::DispenserRequest>(
        "/dispenser_requests", 10,
        std::bind(&HamalTaskDispatcherNode::dispenser_request_callback, this, std::placeholders::_1));

    ingestor_request_sub_ = create_subscription<rmf_ingestor_msgs::msg::IngestorRequest>(
        "/ingestor_requests", 10,
        std::bind(&HamalTaskDispatcherNode::ingestor_request_callback, this, std::placeholders::_1));

    mode_request_pub_ = create_publisher<rmf_fleet_msgs::msg::ModeRequest>(
        "/mode_request", 10);
}


void HamalTaskDispatcherNode::bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg)
{
    bid_responses_[msg->task_id].robot_name = msg->proposal.expected_robot_name;
    bid_responses_[msg->task_id].fleet_name = msg->proposal.fleet_name;
    RCLCPP_INFO(get_logger(), "Received bid response for robot %s, task_id: %s", msg->proposal.expected_robot_name.c_str(), msg->task_id.c_str());
}

void HamalTaskDispatcherNode::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    auto it = bid_responses_.find(task_id);
    if (it != bid_responses_.end())
    {
        it->second.task_type = "pickup";
        auto mode_request_msg = std::make_shared<rmf_fleet_msgs::msg::ModeRequest>();
        mode_request_msg->fleet_name = it->second.fleet_name;
        mode_request_msg->robot_name = it->second.robot_name;
        mode_request_msg->mode.mode = 10; // 10 for pickup
        mode_request_msg->task_id = task_id;
        mode_request_msg->parameters[0].name = "pickup";
        mode_request_msg->parameters[0].value = msg->target_guid;
        mode_request_pub_->publish(*mode_request_msg);

        RCLCPP_INFO(get_logger(), "Published dispenser request for task_id: %s", task_id.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "No bid response found for task_id: %s", task_id.c_str());
    }
}

void HamalTaskDispatcherNode::ingestor_request_callback(const rmf_ingestor_msgs::msg::IngestorRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    auto it = bid_responses_.find(task_id);
    if (it != bid_responses_.end())
    {
        it->second.task_type = "ingestor";

        auto mode_request_msg = std::make_shared<rmf_fleet_msgs::msg::ModeRequest>();
        mode_request_msg->fleet_name = it->second.fleet_name;
        mode_request_msg->robot_name = it->second.robot_name;
        mode_request_msg->mode.mode = 11; // 11 for dropoff
        mode_request_msg->task_id = task_id;
        mode_request_msg->parameters[0].name = "dropoff";
        mode_request_msg->parameters[0].value = msg->target_guid;
        mode_request_pub_->publish(*mode_request_msg);

        RCLCPP_INFO(get_logger(), "Published ingestor request for task_id: %s", task_id.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "No bid response found for task_id: %s", task_id.c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HamalTaskDispatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

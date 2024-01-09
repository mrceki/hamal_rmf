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

    dispenser_result_sub_ = create_subscription<rmf_dispenser_msgs::msg::DispenserResult>(
        "/free_fleet/dispenser_result", 10,
        std::bind(&HamalTaskDispatcherNode::dispenser_result_callback, this, std::placeholders::_1));

    ingestor_result_sub_ = create_subscription<rmf_ingestor_msgs::msg::IngestorResult>(
        "/free_fleet/ingestor_result", 10,
        std::bind(&HamalTaskDispatcherNode::ingestor_result_callback, this, std::placeholders::_1));

    dispenser_request_pub_ = create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
        "/hamal_task_dispatcher/dispenser_request", 10);

    ingestor_request_pub_ = create_publisher<rmf_ingestor_msgs::msg::IngestorRequest>(
        "/hamal_task_dispatcher/ingestor_request", 10);

    dispenser_request_free_pub_ = create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
        "/dispenser_request", 10);

    ingestor_request_free_pub_ = create_publisher<rmf_ingestor_msgs::msg::IngestorRequest>(
        "/ingestor_request", 10);
}

void HamalTaskDispatcherNode::bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg)
{
    bid_responses_[msg->task_id] = msg->proposal.expected_robot_name;
}

void HamalTaskDispatcherNode::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    auto it = bid_responses_.find(task_id.c_str());
    if (it != bid_responses_.end())
    {
        auto new_dispenser_request = std::make_shared<rmf_dispenser_msgs::msg::DispenserRequest>();
        new_dispenser_request->request_guid = it->first;
        new_dispenser_request->target_guid = it->second;
        dispenser_request_pub_->publish(*new_dispenser_request);
    }
    else
    {
        dispenser_request_free_pub_->publish(*msg);
    }
}

void HamalTaskDispatcherNode::ingestor_request_callback(const rmf_ingestor_msgs::msg::IngestorRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    auto it = bid_responses_.find(task_id.c_str());
    if (it != bid_responses_.end())
    {
        auto new_ingestor_request = std::make_shared<rmf_ingestor_msgs::msg::IngestorRequest>();
        new_ingestor_request->request_guid = it->first;
        new_ingestor_request->target_guid = it->second;
        ingestor_request_pub_->publish(*new_ingestor_request);
    }
    else
    {
        ingestor_request_free_pub_->publish(*msg);
    }
}

void HamalTaskDispatcherNode::dispenser_result_callback(const rmf_dispenser_msgs::msg::DispenserResult::SharedPtr msg)
{
    auto new_dispenser_request = std::make_shared<rmf_dispenser_msgs::msg::DispenserRequest>();
    new_dispenser_request->time = msg->time;
    new_dispenser_request->request_guid = msg->request_guid;
    new_dispenser_request->target_guid = msg->source_guid;
    dispenser_request_free_pub_->publish(*new_dispenser_request);
}

void HamalTaskDispatcherNode::ingestor_result_callback(const rmf_ingestor_msgs::msg::IngestorResult::SharedPtr msg)
{
    auto new_ingestor_request = std::make_shared<rmf_ingestor_msgs::msg::IngestorRequest>();
    new_ingestor_request->time = msg->time;
    new_ingestor_request->request_guid = msg->request_guid;
    new_ingestor_request->target_guid = msg->source_guid;
    ingestor_request_free_pub_->publish(*new_ingestor_request);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HamalTaskDispatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

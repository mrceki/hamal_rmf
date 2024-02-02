#include "rmf_hamal/hamal_task_dispatcher.hpp"

HamalTaskDispatcherNode::HamalTaskDispatcherNode() : Node("hamal_task_dispatcher")
{
    task_dispatcher_service_= create_service<rmf_task_msgs::srv::SubmitTask>(
        "/hamal_task_dispatcher/task_info",
        std::bind(&HamalTaskDispatcherNode::dispatch_task_callback, this, std::placeholders::_1, std::placeholders::_2));

    bid_response_sub_ = create_subscription<rmf_task_msgs::msg::BidResponse>(
        "/rmf_task/bid_response", 10,
        std::bind(&HamalTaskDispatcherNode::bid_response_callback, this, std::placeholders::_1));

    dispenser_request_sub_ = create_subscription<rmf_dispenser_msgs::msg::DispenserRequest>(
        "/dispenser_requests", 10,
        std::bind(&HamalTaskDispatcherNode::dispenser_request_callback, this, std::placeholders::_1));

    ingestor_request_sub_ = create_subscription<rmf_ingestor_msgs::msg::IngestorRequest>(
        "/ingestor_requests", 10,
        std::bind(&HamalTaskDispatcherNode::ingestor_request_callback, this, std::placeholders::_1));

    // dispenser_result_sub_ = create_subscription<rmf_dispenser_msgs::msg::DispenserResult>(
    //     "/free_fleet/dispenser_result", 10,
    //     std::bind(&HamalTaskDispatcherNode::dispenser_result_callback, this, std::placeholders::_1));

    // ingestor_result_sub_ = create_subscription<rmf_ingestor_msgs::msg::IngestorResult>(
    //     "/free_fleet/ingestor_result", 10,
    //     std::bind(&HamalTaskDispatcherNode::ingestor_result_callback, this, std::placeholders::_1));

    // dispenser_request_pub_ = create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
    //     "/hamal_task_dispatcher/dispenser_request", 10);

    // ingestor_request_pub_ = create_publisher<rmf_ingestor_msgs::msg::IngestorRequest>(
    //     "/hamal_task_dispatcher/ingestor_request", 10);

    // dispenser_request_free_pub_ = create_publisher<rmf_dispenser_msgs::msg::DispenserRequest>(
    //     "/dispenser_request", 10);

    // ingestor_request_free_pub_ = create_publisher<rmf_ingestor_msgs::msg::IngestorRequest>(
    //     "/ingestor_request", 10);
}

void HamalTaskDispatcherNode::dispatch_task_callback(
    const std::shared_ptr<rmf_task_msgs::srv::SubmitTask::Request> request,
    const std::shared_ptr<rmf_task_msgs::srv::SubmitTask::Response> response)
{
    std::string task_id = request->requester;

    auto it = bid_responses_.find(task_id);
    if (it != bid_responses_.end())
    {
        response->task_id = it->second.task_type;
        response->success = true;
        response->message = "Task submitted successfully";
        RCLCPP_INFO(get_logger(), "Received dispatcher request for robot %s task type: %s", task_id.c_str() ,it->second.task_type.c_str());
    }
    else
    {
        response->task_id = "";
        response->success = false;
        response->message = "Task submission failed";
    }
}

void HamalTaskDispatcherNode::bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg)
{
    bid_responses_[msg->task_id].robot_name = msg->proposal.expected_robot_name;
    RCLCPP_INFO(get_logger(), "Received bid response for robot %s task_id: %s", msg->proposal.expected_robot_name.c_str(), msg->task_id.c_str());
}

void HamalTaskDispatcherNode::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    // Task ID'yi bul
    auto it = bid_responses_.find(task_id);
    if (it != bid_responses_.end())
    {
        it->second.task_type = "dispenser";
        RCLCPP_INFO(get_logger(), "Contents of bid_responses_ map:");
        for (const auto& entry : bid_responses_)
        {
            const std::string& key = entry.first;
            const task_allocation& value = entry.second;

            RCLCPP_INFO(get_logger(), "Key: %s, Task Type: %s, Robot Name: %s",
                        key.c_str(), value.task_type.c_str(), value.robot_name.c_str());
        }
        // auto new_dispenser_request = std::make_shared<rmf_dispenser_msgs::msg::DispenserRequest>(*msg);
        // dispenser_request_pub_->publish(new_dispenser_request);

        RCLCPP_INFO(get_logger(), "Published dispenser request for task_id: %s", task_id.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Task ID not found: %s", task_id.c_str());
    }
}

void HamalTaskDispatcherNode::ingestor_request_callback(const rmf_ingestor_msgs::msg::IngestorRequest::SharedPtr msg)
{
    auto task_id = msg->request_guid;

    auto it = bid_responses_.find(task_id);
    if (it != bid_responses_.end())
    {
        it->second.task_type = "ingestor";

        // auto new_ingestor_request = std::make_shared<rmf_ingestor_msgs::msg::IngestorRequest>(*msg);
        // ingestor_request_pub_->publish(new_ingestor_request);

        RCLCPP_INFO(get_logger(), "Published ingestor request for task_id: %s", task_id.c_str());
    }
    else
    {
        // ingestor_request_free_pub_->publish(*msg);
        RCLCPP_INFO(get_logger(), "Published ingestor request (free) for task_id: %s", task_id.c_str());
    }
}

// void HamalTaskDispatcherNode::dispenser_result_callback(const rmf_dispenser_msgs::msg::DispenserResult::SharedPtr msg)
// {
//     auto new_dispenser_request = std::make_shared<rmf_dispenser_msgs::msg::DispenserRequest>();
//     new_dispenser_request->time = msg->time;
//     new_dispenser_request->request_guid = msg->request_guid;
//     new_dispenser_request->target_guid = msg->source_guid;
//     dispenser_request_free_pub_->publish(*new_dispenser_request);
//     RCLCPP_INFO(get_logger(), "Published dispenser request (free) for task_id: %s", msg->request_guid.c_str());
// }

// void HamalTaskDispatcherNode::ingestor_result_callback(const rmf_ingestor_msgs::msg::IngestorResult::SharedPtr msg)
// {
//     auto new_ingestor_request = std::make_shared<rmf_ingestor_msgs::msg::IngestorRequest>();
//     new_ingestor_request->time = msg->time;
//     new_ingestor_request->request_guid = msg->request_guid;
//     new_ingestor_request->target_guid = msg->source_guid;
//     ingestor_request_free_pub_->publish(*new_ingestor_request);
//     RCLCPP_INFO(get_logger(), "Published ingestor request (free) for task_id: %s", msg->request_guid.c_str());
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HamalTaskDispatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

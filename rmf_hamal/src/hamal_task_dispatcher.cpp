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

    dispenser_result_pub_ = create_publisher<rmf_dispenser_msgs::msg::DispenserResult>(
        "/dispenser_results", 10);
    
    ingestor_result_pub_ = create_publisher<rmf_ingestor_msgs::msg::IngestorResult>(
        "/ingestor_results", 10);

    mode_request_pub_ = create_publisher<rmf_fleet_msgs::msg::ModeRequest>(
        "/robot_mode_requests", 10);
    
    fleet_states_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
        "/fleet_states", 10,
        std::bind(&HamalTaskDispatcherNode::fleet_states_callback, this, std::placeholders::_1));

}


void HamalTaskDispatcherNode::bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg)
{
    bid_responses_[msg->task_id].robot_name = msg->proposal.expected_robot_name;
    bid_responses_[msg->task_id].fleet_name = msg->proposal.fleet_name;
    robot_task_id_[msg->proposal.expected_robot_name] = msg->task_id;
    RCLCPP_INFO(get_logger(), "Received bid response for robot %s, task_id: %s", msg->proposal.expected_robot_name.c_str(), msg->task_id.c_str());
}

void HamalTaskDispatcherNode::fleet_states_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
    for (const auto& robot : msg->robots)
    {
        if (robot_modes_[robot.name] == 10 && robot.mode.mode == 0)
        {
            auto it = robot_task_id_.find(robot.name);
            RCLCPP_WARN(get_logger(), "Task %s found for robot %s", it->second.c_str(), robot.name.c_str());
            auto dispenser_result_msg = std::make_shared<rmf_dispenser_msgs::msg::DispenserResult>();
            dispenser_result_msg->time = now();
            dispenser_result_msg->source_guid = robot.name;
            dispenser_result_msg->request_guid = it->second; // Set the task ID here
            dispenser_result_msg->status = rmf_dispenser_msgs::msg::DispenserResult::SUCCESS; // Set the status here
            dispenser_result_pub_->publish(*dispenser_result_msg);
            RCLCPP_INFO(get_logger(), "Robot %s completed dispenser task", robot.name.c_str());
        }
        if (robot_modes_[robot.name] == 11 && robot.mode.mode == 0)
        {
            auto it = robot_task_id_.find(robot.name);
            auto ingestor_result_msg = std::make_shared<rmf_ingestor_msgs::msg::IngestorResult>();
            ingestor_result_msg->source_guid = robot.name;
            ingestor_result_msg->request_guid = it->second; // Set the task ID here
            ingestor_result_msg->status = rmf_ingestor_msgs::msg::IngestorResult::SUCCESS; // Set the status here
            ingestor_result_pub_->publish(*ingestor_result_msg);
            RCLCPP_INFO(get_logger(), "Robot %s completed ingestor task", robot.name.c_str());
        }
        robot_modes_[robot.name] = robot.mode.mode;
    }
}
void HamalTaskDispatcherNode::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received dispenser request for task_id: %s", msg->request_guid.c_str());
    auto task_id = msg->request_guid;
    auto it = bid_responses_.find(task_id);
    
    if (robot_modes_[it->second.robot_name] == 10 || robot_modes_[it->second.robot_name] == 11)
    {
        RCLCPP_ERROR(get_logger(), "Robot %s is already busy with a task", it->second.robot_name.c_str());
        return;
    }
    else if (it != bid_responses_.end())
    {
        RCLCPP_INFO(get_logger(), "Found bid response for task_id: %s", task_id.c_str());
        it->second.task_type = "pickup";
        auto mode_request_msg = std::make_shared<rmf_fleet_msgs::msg::ModeRequest>();
        mode_request_msg->fleet_name = it->second.fleet_name;
        mode_request_msg->robot_name = it->second.robot_name;
        mode_request_msg->mode.mode = 10; // 10 for pickup
        mode_request_msg->task_id = task_id;
        if (mode_request_msg->parameters.empty()) {
            rmf_fleet_msgs::msg::ModeParameter parameter;
            mode_request_msg->parameters.push_back(parameter);
        }
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
    RCLCPP_INFO(get_logger(), "Found bid response for task_idx: %s", task_id.c_str());
    if (robot_modes_[it->second.robot_name] == 10 || robot_modes_[it->second.robot_name] == 11)
    {
        RCLCPP_ERROR(get_logger(), "Robot %s is already busy with a task", it->second.robot_name.c_str());
        return;
    }
    else if (it != bid_responses_.end())
    {
        it->second.task_type = "ingestor";

        auto mode_request_msg = std::make_shared<rmf_fleet_msgs::msg::ModeRequest>();
        mode_request_msg->fleet_name = it->second.fleet_name;
        mode_request_msg->robot_name = it->second.robot_name;
        mode_request_msg->mode.mode = 11; // 11 for dropoff
        mode_request_msg->task_id = task_id;
        if (mode_request_msg->parameters.empty()) {
            rmf_fleet_msgs::msg::ModeParameter parameter;
            mode_request_msg->parameters.push_back(parameter);
        }
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

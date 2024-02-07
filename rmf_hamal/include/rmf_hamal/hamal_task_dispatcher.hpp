#ifndef HAMAL_TASK_DISPATCHER_NODE_HPP
#define HAMAL_TASK_DISPATCHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "rmf_task_msgs/msg/bid_response.hpp"
#include "rmf_fleet_msgs/msg/mode_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_request.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"

const uint32_t MODE_PICKUP = 10;
const uint32_t MODE_DROPOFF = 11;

class HamalTaskDispatcherNode : public rclcpp::Node
{
public:
  HamalTaskDispatcherNode();

private:
  void bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg);
  void dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg);
  void ingestor_request_callback(const rmf_ingestor_msgs::msg::IngestorRequest::SharedPtr msg);
  void fleet_states_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void publish_mode_request(const std::string& task_id, const std::string& target_guid, uint32_t mode);

  struct task_allocation
  {
    std::string task_type;
    std::string robot_name;
    std::string fleet_name;
  };

  std::unordered_map<std::string, task_allocation> bid_responses_;
  std::unordered_map<std::string, std::uint32_t> robot_modes_;
  std::unordered_map<std::string, std::string> robot_task_id_;

  rclcpp::Subscription<rmf_task_msgs::msg::BidResponse>::SharedPtr bid_response_sub_;
  rclcpp::Subscription<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_sub_;
  rclcpp::Subscription<rmf_ingestor_msgs::msg::IngestorRequest>::SharedPtr ingestor_request_sub_;
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserResult>::SharedPtr dispenser_result_pub_;
  rclcpp::Publisher<rmf_ingestor_msgs::msg::IngestorResult>::SharedPtr ingestor_result_pub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_states_sub_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr mode_request_pub_;
};

#endif

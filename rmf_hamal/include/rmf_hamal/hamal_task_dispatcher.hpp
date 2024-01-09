#ifndef HAMAL_TASK_DISPATCHER_NODE_HPP
#define HAMAL_TASK_DISPATCHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "rmf_task_msgs/msg/bid_response.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_request.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"

class HamalTaskDispatcherNode : public rclcpp::Node
{
public:
  HamalTaskDispatcherNode();

private:
  void bid_response_callback(const rmf_task_msgs::msg::BidResponse::SharedPtr msg);
  void dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::SharedPtr msg);
  void ingestor_request_callback(const rmf_ingestor_msgs::msg::IngestorRequest::SharedPtr msg);
  void dispenser_result_callback(const rmf_dispenser_msgs::msg::DispenserResult::SharedPtr msg);
  void ingestor_result_callback(const rmf_ingestor_msgs::msg::IngestorResult::SharedPtr msg);

  std::unordered_map<std::string, std::string> bid_responses_;

  rclcpp::Subscription<rmf_task_msgs::msg::BidResponse>::SharedPtr bid_response_sub_;
  rclcpp::Subscription<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_sub_;
  rclcpp::Subscription<rmf_ingestor_msgs::msg::IngestorRequest>::SharedPtr ingestor_request_sub_;
  rclcpp::Subscription<rmf_dispenser_msgs::msg::DispenserResult>::SharedPtr dispenser_result_sub_;
  rclcpp::Subscription<rmf_ingestor_msgs::msg::IngestorResult>::SharedPtr ingestor_result_sub_;

  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_pub_;
  rclcpp::Publisher<rmf_ingestor_msgs::msg::IngestorRequest>::SharedPtr ingestor_request_pub_;
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_free_pub_;
  rclcpp::Publisher<rmf_ingestor_msgs::msg::IngestorRequest>::SharedPtr ingestor_request_free_pub_;
};

#endif
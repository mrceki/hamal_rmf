#include <ros/ros.h>
#include <hamal_ff_client/DockingBt.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <bt_amr/print_dolly.h>
#include <bt_amr/dolly_client.h>
#include <bt_amr/stop_dolly_client.h>
#include <bt_amr/docking.h>
#include <bt_amr/move_base_client.h>
#include <bt_amr/battery_check.h>
#include <bt_amr/is_battery_charged.h>

using namespace BT;
BehaviorTreeFactory factory;

bool trigger_docking(hamal_ff_client::DockingBt::Request& req, hamal_ff_client::DockingBt::Response& res){
  factory.registerNodeType<FindDolly>("FindDolly");
  factory.registerNodeType<AutoDockAction>("AutoDockAction");
  factory.registerNodeType<StopDolly>("StopDolly");
  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<BatteryCheck>("BatteryCheck");
  factory.registerNodeType<IsBatteryCharged>("IsBatteryCharged");
  auto tree = factory.createTreeFromFile("/home/cenk/Desktop/docking2.xml"); 
  
  StdCoutLogger logger_cout(tree);
  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();
  return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "docking_bt_service");
    ros::NodeHandle nh;
    ros::ServiceServer docking_service = nh.advertiseService("docking_bt", trigger_docking);
    ros::spin();

    return 0;
}

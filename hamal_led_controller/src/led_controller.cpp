#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>

ros::Publisher led_pub;
std_msgs::Int32 m_led_status;
int taskMode;
float max_linear_vel = 0.25;
float max_angular_vel = 0.15;
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
    {
        m_led_status.data = 7;
    }
}

// uint8 PENDING         = 0   # The goal has yet to be processed by the action server
// uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
// uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
//                             #   and has since completed its execution (Terminal State)
// uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
// uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
//                             #    to some failure (Terminal State)
// uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
//                             #    because the goal was unattainable or invalid (Terminal State)
// uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
//                             #    and has not yet completed execution
// uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
//                             #    but the action server has not yet confirmed that the goal is canceled
// uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
//                             #    and was successfully cancelled (Terminal State)
// uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
//                             #    sent over the wire by an action server

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    if(m_led_status.data == 1 || m_led_status.data == 2){
        return;
    }
    if (msg->angular.z > max_angular_vel*0.75){
        m_led_status.data = 5;
    }
    else if(msg->angular.z < -max_angular_vel * 0.75){
        m_led_status.data = 6;
    }
    else if(abs(max_linear_vel) < abs(msg->angular.z * 1.5)){
        m_led_status.data = msg->angular.z > 0 ? 5 : 6;
    }
    else if(msg->linear.x != 0){
        m_led_status.data = msg->linear.x > 0 ? 3 : 4;
    }
    else if(msg->angular.z != 0){
        m_led_status.data = msg->angular.z > 0 ? 5 : 6;
    }
    else{
        m_led_status.data = 0;
    }

    led_pub.publish(m_led_status);
}

void dockingCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(m_led_status.data != 2){
        if(msg->status_list[0].status==actionlib_msgs::GoalStatus::ACTIVE){
            m_led_status.data = 1;
            led_pub.publish(m_led_status);
        }
        else{
            m_led_status.data = 0;
            led_pub.publish(m_led_status);
        }
    }
}

// void hardwareStatusCallback(const ){
//     m_led_status.data
// }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_controller");
    ros::NodeHandle nh;

    led_pub = nh.advertise<std_msgs::Int32>("led_status", 10);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    ros::Subscriber battery_sub = nh.subscribe("bms_battery", 10, batteryCallback);
    ros::Subscriber autodock_status_sub = nh.subscribe("autodock_action/status", 10, dockingCallback);
    ros::Subscriber hardware_status_sub = nh.subscribe("/hamal/hardware_status", 10, hardwareStatusCallback);


    ros::spin();

    return 0;
}



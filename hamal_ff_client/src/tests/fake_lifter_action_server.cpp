#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <hamal_custom_interfaces/LifterOperationAction.h>

class FakeLifterActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<hamal_custom_interfaces::LifterOperationAction> as_;
    std::string action_name_;
    hamal_custom_interfaces::LifterOperationFeedback feedback_;
    hamal_custom_interfaces::LifterOperationResult result_;
    float current_position_;

public:
    FakeLifterActionServer(std::string name) :
        as_(nh_, name, boost::bind(&FakeLifterActionServer::executeCB, this, _1), false),
        action_name_(name),
        current_position_(0.0)
    {
        as_.start();
    }

    ~FakeLifterActionServer(void) {}

    void executeCB(const hamal_custom_interfaces::LifterOperationGoalConstPtr &goal) {
        ros::Rate loop_rate(1000); 
        float target_position = goal->target_position;
        float profile_time = 5.0; // goal->profile_time
        float increment = (target_position - current_position_) / (profile_time * loop_rate.expectedCycleTime().toSec());
        
        ROS_INFO("Fake Lifter Action Server has received a goal: target_position = %f, profile_time = %f",
                 target_position, profile_time);
    
        float time_elapsed = 0.0;
        while (time_elapsed < profile_time) {
            feedback_.current_position = current_position_;
            feedback_.target_command = target_position;
    
            as_.publishFeedback(feedback_);
    
            if (fabs(target_position - current_position_) < 0.01) {
                current_position_ = target_position;    
                break;
            }
    
            current_position_ += increment;
            if (current_position_ < target_position || current_position_ > target_position) {
                time_elapsed += loop_rate.expectedCycleTime().toSec();
                loop_rate.sleep();
            } else {
                current_position_ = target_position;
                break;
            }
        }
    
        if (fabs(target_position - current_position_) < 0.01) {
            result_.target_reached = true;
            result_.status_str = "Target position reached.";
            ROS_INFO("Target position reached.");
        } else {
            result_.target_reached = false;
            result_.status_str = "Target position not reached within profile time.";
            ROS_INFO("Target position not reached within profile time.");
        }
    
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_lifter_action_server");

    FakeLifterActionServer fake_server("fake_lifter_server");
    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_goal");
    // Spin a thread
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move_base server");

    //  create a goal to send to move_base
    move_base_msgs::MoveBaseGoal goal;

    int waypoints[5][2] = {{-480,180}, {-495,200}, {-460,200}, {-474,231}, {-446,226}};

    for(int i=0; i<5; ++i){
        // Send goal pose
        goal.target_pose.header.frame_id = "wamv/odom";
        goal.target_pose.header.stamp = ros::Time::now();

        // Do NOT modify the following for final submission.
        goal.target_pose.pose.position.x = waypoints[i][0];
        goal.target_pose.pose.position.y = waypoints[i][1];

        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 0.0;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Excellent! Your robot has reached the goal position.");
        else
            ROS_INFO("The robot failed to reach the goal position");
    }

    return 0;
}

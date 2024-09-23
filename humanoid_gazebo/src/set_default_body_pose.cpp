#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "set_default_body_pose");
    ros::NodeHandle nh;

    // Get robot name from parameter server
    std::string robot_name;
    ros::param::get("/robot_name", robot_name);

    // Create publisher for moving robot
    ros::Publisher move_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

    // Create model state message
    gazebo_msgs::ModelState model_state_msg;
    model_state_msg.model_name = robot_name;
    model_state_msg.reference_frame = "world";
    model_state_msg.pose.position.x = 0.0;
    model_state_msg.pose.position.y = 0.0;
    model_state_msg.pose.position.z = 0.685;
    model_state_msg.pose.orientation.x = 0.0;
    model_state_msg.pose.orientation.y = 0.0;
    model_state_msg.pose.orientation.z = 0.0;
    model_state_msg.pose.orientation.w = 1.0;

    // Publish model state message
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        move_pub.publish(model_state_msg);
        loop_rate.sleep();
    }

    return 0;
}
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>

int main(int argc, char **argv) {
    // Set target joint positions
    double joint_pos[10] = {0.0, 0.0, -8 * M_PI / 180.0, 23.9 * M_PI / 180.0, -15.9 * M_PI / 180.0,
                            0.0, 0.0, -8 * M_PI / 180.0, 23.9 * M_PI / 180.0, -15.9 * M_PI / 180.0};

    // Initialize ROS
    ros::init(argc, argv, "set_default_joint_pose");
    ros::NodeHandle nh;

    // Get robot name from parameter server
    std::string robot_name;
    ros::param::get("/robot_name", robot_name);

    // Create publishers for each servo
    ros::Publisher servo_pub[10];
    servo_pub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_yaw_controller/command", 1);
    servo_pub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_abad_controller/command", 1);
    servo_pub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_hip_pitch_controller/command", 1);
    servo_pub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_knee_controller/command", 1);
    servo_pub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/left_ankle_controller/command", 1);
    servo_pub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_yaw_controller/command", 1);
    servo_pub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_abad_controller/command", 1);
    servo_pub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_hip_pitch_controller/command", 1);
    servo_pub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_knee_controller/command", 1);
    servo_pub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "/right_ankle_controller/command", 1);

    // Create low command message
    unitree_legged_msgs::LowCmd low_cmd;
    for (int i = 0; i < 10; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = joint_pos[i];
        low_cmd.motorCmd[i].Kp = 500;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kd = 15;
        low_cmd.motorCmd[i].tau = 0;
    }

    // Publish low command message
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        for (int i = 0; i < 10; i++) {
            servo_pub[i].publish(low_cmd.motorCmd[i]);
        }
        loop_rate.sleep();
    }

    return 0;
}
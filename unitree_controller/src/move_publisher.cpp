#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <stdio.h>
#include <tf/transform_datatypes.h>
// #include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>

double rwx_pos = 0.0;
double rwy_pos = 0.0;

double rwx_vel = 0.0;
double rwy_vel = 0.0;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    rwx_pos = msg->position[12];
    rwx_vel = msg->velocity[12];

    rwy_pos = msg->position[13];
    rwy_vel = msg->velocity[13];
}

int main(int argc, char **argv)
{
    enum coord
    {
        WORLD,
        ROBOT
    };
    coord def_frame = coord::WORLD;

    ros::init(argc, argv, "move_publisher");
    ros::NodeHandle nh;

    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;

    ros::Subscriber joint_state_sub = nh.subscribe(robot_name + "_gazebo/joint_states", 1000, joint_state_callback);

    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    ros::Publisher rwx_cmd_publisher = nh.advertise<std_msgs::Float64>(robot_name + "_gazebo/RWX_controller/command", 1000);
    ros::Publisher rwy_cmd_publisher = nh.advertise<std_msgs::Float64>(robot_name + "_gazebo/RWY_controller/command", 1000);
    ros::Publisher tgt_pose_publisher = nh.advertise<geometry_msgs::Pose>(robot_name + "_gazebo/target_pose", 1000);

    std::srand(std::time(nullptr));

    auto random_double = [](double min, double max)
    {
        return min + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (max - min)));
    };

    gazebo_msgs::ModelState model_state_pub;
    std_msgs::Float64 rwx_cmd;
    std_msgs::Float64 rwy_cmd;
    geometry_msgs::Pose tgt_pose;

    // D control for RW
    double Kd = 0.5;
    rwx_cmd.data = -Kd * rwx_vel;
    rwy_cmd.data = -Kd * rwy_vel;    

    model_state_pub.model_name = robot_name + "_gazebo";
    model_state_pub.pose.position.x = 0.0;
    model_state_pub.pose.position.y = 0.0;
    model_state_pub.pose.position.z = 1.75;

    ros::Rate loop_rate(1000);

    if (def_frame == coord::WORLD)
    {
        // Generate random orientation
        double roll = random_double(-M_PI, M_PI);
        double pitch = random_double(-M_PI, M_PI);
        double yaw = random_double(-M_PI, M_PI);

        std::cout << "Random initial attitude" << std::endl;
        std::cout << "Roll: " << roll << std::endl;
        std::cout << "Pitch: " << pitch << std::endl;
        std::cout << "Yaw: " << yaw << std::endl;

        model_state_pub.reference_frame = "world";
        tf::Quaternion q;
        model_state_pub.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

        // Get the best falling orientation
        double z0 = model_state_pub.pose.orientation.z;
        double w0 = model_state_pub.pose.orientation.w;
        double z_tgt = z0 / sqrt(z0 * z0 + w0 * w0);
        double w_tgt = w0 / sqrt(z0 * z0 + w0 * w0);
        tgt_pose.orientation.z = z_tgt;
        tgt_pose.orientation.w = w_tgt;

        while (ros::ok())
        {
            move_publisher.publish(model_state_pub);
            rwx_cmd_publisher.publish(rwx_cmd);
            rwy_cmd_publisher.publish(rwy_cmd);
            tgt_pose_publisher.publish(tgt_pose);

            loop_rate.sleep();
        }
    }
    else if (def_frame == coord::ROBOT)
    {
        model_state_pub.twist.linear.x = 0.02; // 0.02: 2cm/sec
        model_state_pub.twist.linear.y = 0.0;
        model_state_pub.twist.linear.z = 0.08;

        model_state_pub.twist.angular.x = 0.0;
        model_state_pub.twist.angular.y = 0.0;
        model_state_pub.twist.angular.z = 0.0;

        model_state_pub.reference_frame = "base";

        while (ros::ok())
        {
            move_publisher.publish(model_state_pub);
            loop_rate.sleep();
        }
    }
}

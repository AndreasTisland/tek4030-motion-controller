#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/server.h>
#include <motion_control/pd_paramsConfig.h>

#include <crustcrawler_lib/dynamics_simple_6dof.h>

#include <iostream>

using namespace crustcrawler_lib;
using namespace Eigen;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& joint_states);
void apply_pd_control();

ros::Publisher* joint1_command_pub_global;
ros::Publisher* joint2_command_pub_global;
ros::Publisher* joint3_command_pub_global;
ros::Publisher* joint4_command_pub_global;
ros::Publisher* joint5_command_pub_global;
ros::Publisher* joint6_command_pub_global;

DynamicsSimple6DOF dynamics;

double k_p = 0;
double k_d = 0;

VectorXd q_d(6); 
VectorXd q(6);
VectorXd q_dot(6);
VectorXd K_P(6);
VectorXd K_D(6);

void pd_reconfigure_callback(motion_control::pd_paramsConfig &config, uint32_t level) {
    k_p = config.kp;
    k_d = config.kd;
    
    ROS_INFO("PD-reconfigure: [Kp=%f] - [Kd=%f]",k_p,k_d);
    
    K_P.setConstant(k_p);
    K_D.setConstant(k_d);
}

void setpoint_callback(const std_msgs::Float64MultiArray::ConstPtr& setpoint) {
    ROS_INFO("New setpoint...");
    for (int i = 0; i < 6; i++) {
        q_d(i) = setpoint->data[i];
        ROS_INFO("q_d[%d]=%f",i,q_d(i));
    }
    
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& joint_states) {
    
    int t = 0;
    for(int i = 0; i < q.size(); i++){
        q(i) = joint_states->position[i+t];
        q_dot(i) = joint_states->velocity[i+t];
    }
    
    apply_pd_control();
    
}

void apply_pd_control() {
    VectorXd q_error(6); 
    q_error = q_d-q;
    
    VectorXd gravity(6);
    gravity = dynamics.getGravityVector(q);
    
    VectorXd u(6);
    u = K_P.asDiagonal()*q_error - K_D.asDiagonal()*q_dot + gravity;

    std_msgs::Float64 ui;
    ui.data = u(0);
    joint1_command_pub_global->publish(ui);
    ui.data = u(1);
    joint2_command_pub_global->publish(ui);
    ui.data = u(2);
    joint3_command_pub_global->publish(ui);
    ui.data = u(3);
    joint4_command_pub_global->publish(ui);
    ui.data = u(4);
    joint5_command_pub_global->publish(ui);
    ui.data = u(5);
    joint6_command_pub_global->publish(ui);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pd_gravity_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber joint_state_sub = nh.subscribe("/crustcrawler/joint_states", 1000, joint_state_callback);
    ros::Subscriber setpoint_sub = nh.subscribe("/motion_control/setpoint", 1, setpoint_callback);
    
    ros::Publisher joint1_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint1_controller/command", 1);
    ros::Publisher joint2_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint2_controller/command", 1); 
    ros::Publisher joint3_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint3_controller/command", 1); 
    ros::Publisher joint4_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint4_controller/command", 1); 
    ros::Publisher joint5_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint5_controller/command", 1);
    ros::Publisher joint6_command_pub = nh.advertise<std_msgs::Float64>("/crustcrawler/joint6_controller/command", 1); 
    
    joint1_command_pub_global = &joint1_command_pub;
    joint2_command_pub_global = &joint2_command_pub;
    joint3_command_pub_global = &joint3_command_pub;
    joint4_command_pub_global = &joint4_command_pub;
    joint5_command_pub_global = &joint5_command_pub;
    joint6_command_pub_global = &joint6_command_pub;
    
    
    // Initiate dynamic reconfigure
    dynamic_reconfigure::Server<motion_control::pd_paramsConfig> server;
    dynamic_reconfigure::Server<motion_control::pd_paramsConfig>::CallbackType f;
    
    f = boost::bind(&pd_reconfigure_callback, _1, _2);
    server.setCallback(f);
    
    // Getting control parameters from launch file
    nh.param("kp", k_p, 2.0);
    nh.param("kd", k_d, 1.0);
    
    K_P.setConstant(k_p);
    K_D.setConstant(k_d);
    
    q_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
    
    ros::spin();

    return 0;
}

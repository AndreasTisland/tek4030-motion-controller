#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <motion_control/set_qdConfig.h>

#include <iostream>

ros::Publisher *setpoint_pub_global;

std_msgs::Float64MultiArray setpoint;
Eigen::VectorXd q_d(6);
bool setpoint_config = false;


void setpoint_reconfigure_callback(motion_control::set_qdConfig &config, uint32_t level) {
    ROS_INFO("Setpoint-reconfigure");
    if (setpoint_config) {
        q_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        q_d(1) = config.set_joint2; 
        setpoint.data[1] = q_d(1);
        setpoint_pub_global->publish(setpoint);
    } else {
        setpoint_config = true;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "setpoint_node");
    ros::NodeHandle nh;
    
    ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float64MultiArray>("/motion_control/setpoint",1);
    setpoint_pub_global = &setpoint_pub;
    
    
    // Initiate dynamic reconfigure
    dynamic_reconfigure::Server<motion_control::set_qdConfig> server;
    dynamic_reconfigure::Server<motion_control::set_qdConfig>::CallbackType f;
    
    f = boost::bind(&setpoint_reconfigure_callback, _1, _2);
    server.setCallback(f);
    
    q_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 

    for (int i = 0; i < 6; i++) {
        setpoint.data.push_back(0.0);
        setpoint.data[i] = (q_d(i));
    }
    
    ros::spin();
    
    return 0;
}

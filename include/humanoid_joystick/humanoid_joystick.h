#ifndef HUMANOID_JOYSTICK_H
#define HUMANOID_JOYSTICK_H

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

namespace joystick_control {
class HumanoidJoystick {
public:
    HumanoidJoystick()
        : error_(false),
          received_init_state_(false),
          axis_switched_(false),
          axis_btn_freed_(true),
          update_rate_(0)
    {
        trans_speed_ = Eigen::Vector3d::Zero();
        rot_speed_ = Eigen::Matrix3d::Identity();
        finger_positions_ = Eigen::Vector3d::Zero();
        finger_velocities_ = Eigen::Vector3d::Zero();
    }
    bool start(ros::NodeHandle& nh, double update_rate);
    void publishCommand();
private:
    void joyCB(const sensor_msgs::JoyConstPtr& joy_ptr);
    void stateCB(const geometry_msgs::PoseStampedConstPtr& pose_ptr_);
    void loadParam(ros::NodeHandle& nh, std::string key, std::string& value);
    void loadKeybinding(ros::NodeHandle& nh, std::string key, int& value);
    void updateCmd();
    void moveHand(const Eigen::Vector3d& finger_positions);
    ros::NodeHandle nh_;
    ros::NodeHandle joystick_nh_;

    ros::Subscriber joy_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher cart_cmd_pub_;
    ros::Publisher head_traj_pub_;
    ros::Publisher reset_ft_pub_;
    ros::Publisher toggle_compl_pub_;

    ros::Publisher thumb_pos_pub_;
    ros::Publisher index_pos_pub_;
    ros::Publisher middle_pos_pub_;

    bool error_;
    bool received_init_state_;
    // Parameters
    std::string controller_name_;
    std::string reset_ft_topic_;
    std::string toggle_compl_topic_;
    std::string cart_cmd_topic_;
    std::string head_cmd_topic_;
    std::string state_topic_;
    std::string thumb_topic_;
    std::string index_topic_;
    std::string middle_topic_;

    int cart_trans_x_;
    int cart_trans_y_;
    int cart_rot_x_;
    int cart_rot_y_;
    int head_pan_;
    int head_tilt_;
    int switch_axis_btn_;
    int hello_btn_;
    int shake_btn_;
    int open_hand_btn_;
    int close_hand_btn_;
    int reset_ft_btn_;
    int toggle_compl_btn_;

    Eigen::Affine3d state_cmd_;
    std::string frame_id_;

    Eigen::Vector3d trans_speed_;
    Eigen::Matrix3d rot_speed_;

    Eigen::Vector3d finger_positions_;
    Eigen::Vector3d finger_velocities_;

    bool axis_switched_;
    bool axis_btn_freed_;

    double update_rate_;
};
}

#endif

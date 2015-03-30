#include <humanoid_joystick/humanoid_joystick.h>

#include <eigen_conversions/eigen_msg.h>

namespace joystick_control {

bool HumanoidJoystick::start(ros::NodeHandle& nh, double update_rate) {
    // Load parameters
    nh_ = nh;
    update_rate_ = update_rate;
    loadParam(nh, "controller_name", controller_name_);
    loadParam(nh, "reset_ft_topic", reset_ft_topic_);
    loadParam(nh, "toggle_compl_topic", toggle_compl_topic_);
    loadParam(nh, "cart_cmd_topic", cart_cmd_topic_);
    loadParam(nh, "head_cmd_topic", head_cmd_topic_);
    loadParam(nh, "state_topic", state_topic_);
    loadParam(nh, "thumb_topic", thumb_topic_);
    loadParam(nh, "middle_topic", middle_topic_);
    loadParam(nh, "index_topic", index_topic_);
    joystick_nh_ = ros::NodeHandle(nh_, controller_name_ + "_keybindings");
    loadKeybinding(joystick_nh_, "cart_trans_x", cart_trans_x_);
    loadKeybinding(joystick_nh_, "cart_trans_y", cart_trans_y_);
    loadKeybinding(joystick_nh_, "cart_rot_x", cart_rot_x_);
    loadKeybinding(joystick_nh_, "cart_rot_y", cart_rot_y_);
    loadKeybinding(joystick_nh_, "head_pan", head_pan_);
    loadKeybinding(joystick_nh_, "head_tilt", head_tilt_);
    loadKeybinding(joystick_nh_, "switch_axis_btn", switch_axis_btn_);
    loadKeybinding(joystick_nh_, "hello_btn", hello_btn_);
    loadKeybinding(joystick_nh_, "shake_btn", shake_btn_);
    loadKeybinding(joystick_nh_, "open_hand_btn", open_hand_btn_);
    loadKeybinding(joystick_nh_, "close_hand_btn", close_hand_btn_);
    loadKeybinding(joystick_nh_, "reset_ft_btn", reset_ft_btn_);
    loadKeybinding(joystick_nh_, "toggle_compl_btn", toggle_compl_btn_);

    // Topics
    if (!error_) {
        joy_sub_ = nh_.subscribe("/joy",10, &HumanoidJoystick::joyCB, this);
        state_sub_ = nh_.subscribe(state_topic_, 10, &HumanoidJoystick::stateCB, this);
        cart_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cart_cmd_topic_, 10);
        reset_ft_pub_ = nh_.advertise<std_msgs::Empty>(reset_ft_topic_, 1);
        toggle_compl_pub_ = nh.advertise<std_msgs::Empty>(toggle_compl_topic_, 10);
        thumb_pos_pub_ = nh.advertise<std_msgs::Float64>(thumb_topic_, 10);
        index_pos_pub_ = nh.advertise<std_msgs::Float64>(middle_topic_, 10);
        middle_pos_pub_ = nh.advertise<std_msgs::Float64>(index_topic_, 10);
    }

    return !error_;
}

void HumanoidJoystick::publishCommand() {
    if (received_init_state_) {
        updateCmd();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id_;
        pose.header.stamp = ros::Time::now();
        tf::poseEigenToMsg(state_cmd_, pose.pose);
        cart_cmd_pub_.publish(pose);
        moveHand(finger_positions_);
    }
}

void HumanoidJoystick::stateCB(const geometry_msgs::PoseStampedConstPtr& pose_ptr_) {
    if (!received_init_state_) {
        ROS_INFO_STREAM("Init state received.");
        received_init_state_ = true;
        tf::poseMsgToEigen(pose_ptr_->pose, state_cmd_);
        frame_id_ = pose_ptr_->header.frame_id;
    }
}

void HumanoidJoystick::updateCmd() {
    state_cmd_.translation() += trans_speed_;
    state_cmd_.rotate(rot_speed_);
    finger_positions_ += finger_velocities_;
}

void HumanoidJoystick::moveHand(const Eigen::Vector3d& finger_positions) {
    std_msgs::Float64 thumb, middle, index;
    thumb.data = finger_positions(0);
    middle.data = finger_positions(1);
    index.data = finger_positions(2);

    thumb_pos_pub_.publish(thumb);
    middle_pos_pub_.publish(middle);
    index_pos_pub_.publish(index);
    ROS_INFO_STREAM_THROTTLE(0.5, "thumb: " << finger_positions(0) << ", middle: " << finger_positions(1) << ", index: " << finger_positions(2));
}

void HumanoidJoystick::joyCB(const sensor_msgs::JoyConstPtr& joy_ptr) {
    if (!received_init_state_) {
        return;
    }
    if (joy_ptr->buttons[switch_axis_btn_] && axis_btn_freed_) {
        axis_switched_ = !axis_switched_;
        axis_btn_freed_ = false;
    }
    if (!joy_ptr->buttons[switch_axis_btn_]) {
        axis_btn_freed_ = true;
    }
    double trans_speed_factor = 0.07; // TODO adjust speed factor
    double rot_speed_factor = 0.5;
    double roll = 0, pitch = 0, yaw = 0;
    if (!axis_switched_) {
        trans_speed_(0) = joy_ptr->axes[cart_trans_y_] * trans_speed_factor;
        trans_speed_(1) = joy_ptr->axes[cart_trans_x_] * trans_speed_factor;
        trans_speed_(2) = 0;
        yaw = 0;
        pitch = -joy_ptr->axes[cart_rot_y_] * rot_speed_factor;
        roll = joy_ptr->axes[cart_rot_x_] * rot_speed_factor;
    } else {
        trans_speed_(0) = 0;
        trans_speed_(1) = joy_ptr->axes[cart_trans_x_] * trans_speed_factor;
        trans_speed_(2) = joy_ptr->axes[cart_trans_y_] * trans_speed_factor;
        yaw = joy_ptr->axes[cart_rot_x_] * rot_speed_factor;
        pitch = -joy_ptr->axes[cart_rot_y_] * rot_speed_factor;
        roll = 0;
    }
    trans_speed_ *= update_rate_;
    rot_speed_ = Eigen::AngleAxisd(roll*update_rate_, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(pitch*update_rate_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw*update_rate_, Eigen::Vector3d::UnitX());

    // Hand
    if (joy_ptr->buttons[open_hand_btn_]) {
        Eigen::Vector3d opening_speed(-0.1, 0.1, -0.1);
        finger_velocities_ = update_rate_ * opening_speed;
    } else {
        if (joy_ptr->buttons[close_hand_btn_]) {
            Eigen::Vector3d closing_speed(0.1, -0.1, 0.1);
            finger_velocities_ = update_rate_ * closing_speed;
        } else {
            finger_velocities_ = Eigen::Vector3d::Zero();
        }
    }
}

void HumanoidJoystick::loadParam(ros::NodeHandle &nh, std::string key, std::string& value) {
    if (!nh.getParam(key, value)) {
        ROS_ERROR_STREAM("Could not find key '" << key << "' in namespace '" << nh.getNamespace() << "'.");
        error_ = true;
    }
}

void HumanoidJoystick::loadKeybinding(ros::NodeHandle& nh, std::string key, int& value) {
    if (!nh.getParam(key, value)) {
        ROS_ERROR_STREAM("Could not find key '" << key << "' in namespace '" << nh.getNamespace() << "'.");
        error_ = true;
    }
    if (value < 0) {
        ROS_ERROR_STREAM("Param " << key << " is negative.");
        error_ = true;
    }
}

}

#include <humanoid_joystick/humanoid_joystick.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    ros::NodeHandle nh("humanoid_joystick");
    ROS_INFO_STREAM("Humanoid joystick node started.");
    joystick_control::HumanoidJoystick joystick;
    if(joystick.start(nh, 0.008)) {
            ROS_INFO_STREAM("Humanoid joystick control initialised.2");
        ros::Rate rate(125);
        while (ros::ok()) {
            ros::spinOnce();
            joystick.publishCommand();
            rate.sleep();
        }
    } else {
        ROS_ERROR_STREAM("Starting joystick server failed. Shutting down node.");
        return 0;
    }
    return 0;
}

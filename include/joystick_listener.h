#ifndef JOYSTICK_LISTENER_H
#define JOYSTICK_LISTENER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

enum class JoyButton {
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START,
    XBOX_BUTTON,
    BUTTON_STICK_LEFT,
    BUTTON_STICK_RIGHT,
    DPAD_LEFT,
    DPAD_RIGHT,
    DPAD_UP,
    DPAD_DOWN,
    COUNT
};

enum class JoyAxis {
    LEFT_HORIZONTAL,
    LEFT_VERTICAL,
    LT,
    RIGHT_HORIZONTAL,
    RIGHT_VERTICAL,
    RT,
    DPAD_HORIZONTAL,
    DPAD_VERTICAL,
    COUNT
};

enum class JoyButtonState {
    UP, // Released, event was processed
    PRESSED, // Pressed down, not processed yet
    DOWN, // Pressed, event was processed
    RELEASED // Released, event not processed
};

class JoystickListener {
public:
    JoystickListener(ros::NodeHandle &n);

    void callback(const sensor_msgs::Joy joy_msg);
    bool query_button_value(JoyButton joy_button);
    JoyButtonState query_button_state(JoyButton joy_button);
    double query_axis(JoyAxis joy_axis);

private:
    ros::Subscriber joy_sub;
    sensor_msgs::Joy joy_msg;
    std::vector<JoyButtonState> joy_button_states;
    bool lt_pressed, rt_pressed;
};

#endif

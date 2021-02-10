#include "joystick_listener.h"

JoystickListener::JoystickListener(ros::NodeHandle &n)
{
    joy_sub = n.subscribe("joy", 1, &JoystickListener::callback, this);
    joy_msg.buttons.resize(std::size_t(JoyButton::COUNT));
    joy_msg.axes.resize(std::size_t(JoyAxis::COUNT));
    joy_msg.axes[std::size_t(JoyAxis::LT)] = 1.0;
    joy_msg.axes[std::size_t(JoyAxis::RT)] = 1.0;

    joy_button_states.resize(std::size_t(JoyButton::COUNT));
}

void JoystickListener::callback(const sensor_msgs::Joy joy_msg)
{
    std::copy(
        joy_msg.buttons.cbegin(), joy_msg.buttons.cend(),
        this->joy_msg.buttons.begin()
    );
    std::copy(
        joy_msg.axes.cbegin(), joy_msg.axes.cend(),
        this->joy_msg.axes.begin()
    );

    if (!lt_pressed) {
        if (joy_msg.axes[std::size_t(JoyAxis::LT)] == 0) {
            this->joy_msg.axes[std::size_t(JoyAxis::LT)] = 1.0;
        } else {
            lt_pressed = true;
        }
    }
    if (!rt_pressed) {
        if (joy_msg.axes[std::size_t(JoyAxis::RT)] == 0) {
            this->joy_msg.axes[std::size_t(JoyAxis::RT)] = 1.0;
        } else {
            rt_pressed = true;
        }
    }

    // Update the state of the button to the PRESSED or RELEASED state
    // Only changes to DOWN or UP states when the event is processed.
    // If the state of the button isn't queried, then it will stay in the same
    // state, but this doesn't matter if the state isn't being checked.
    for (std::size_t i=0; i < joy_button_states.size(); i++) {
        if (joy_msg.buttons[i] &&
                joy_button_states[i] == JoyButtonState::UP) {
            joy_button_states[i] = JoyButtonState::PRESSED;
        } else if (!joy_msg.buttons[i] &&
                joy_button_states[i] == JoyButtonState::DOWN) {
            joy_button_states[i] = JoyButtonState::RELEASED;
        }
    }
}

bool JoystickListener::query_button_value(JoyButton joy_button)
{
    return joy_msg.buttons[std::size_t(joy_button)];
}

JoyButtonState JoystickListener::query_button_state(JoyButton joy_button)
{
    JoyButtonState state = joy_button_states[std::size_t(joy_button)];
    if (state == JoyButtonState::PRESSED) {
        joy_button_states[std::size_t(joy_button)] = JoyButtonState::DOWN;
    } else if (state == JoyButtonState::RELEASED) {
        joy_button_states[std::size_t(joy_button)] = JoyButtonState::UP;
    }
    return state;
}

double JoystickListener::query_axis(JoyAxis joy_axis)
{
    return joy_msg.axes[std::size_t(joy_axis)];
}

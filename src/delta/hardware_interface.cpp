#include <cmath>
#include <vector>
#include <stdint.h>

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Hardware: public hardware_interface::RobotHW {
public:
    Hardware(ros::NodeHandle &n)
    {
        names = {"theta_1", "theta_2", "theta_3", "theta_4", "gripper" };
        pos.resize(names.size());
        vel.resize(names.size());
        eff.resize(names.size());
        cmd.resize(names.size());

        for (std::size_t i = 0; i < names.size(); i++) {
            state_interface.registerHandle(hardware_interface::JointStateHandle(
                names[i], &pos[i], &vel[i], &eff[i]
            ));
            cmd_interface.registerHandle(hardware_interface::JointHandle(
                state_interface.getHandle(names[i]), &cmd[i]
            ));
        }
        registerInterface(&state_interface);
        registerInterface(&cmd_interface);
    }

    ~Hardware()
    {
    }

    void read()
    {
        pos = cmd;
    }

    void write()
    {
        for (std::size_t i = 0; i < cmd.size(); i++) {
            double angle = cmd[i];
            while (angle < 0) angle += 2*M_PI;
            while (angle > 2*M_PI) angle -= 2*M_PI;
            uint16_t cmd = (angle/(2*M_PI))*65536;
        }
    }

private:
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::PositionJointInterface cmd_interface;

    std::vector<std::string> names;
    std::vector<double> pos, vel, eff, cmd;
};

class Node {
public:
    Node(ros::NodeHandle& n): hw(n), cm(&hw, n)
    {
        loop_timer = n.createTimer(
            ros::Duration(1.0/50),
            &Node::loop,
            this
        );
    }

    void loop(const ros::TimerEvent &timer)
    {
        hw.read();
        cm.update(
            ros::Time::now(),
            ros::Duration(timer.current_real - timer.last_real)
        );
        hw.write();
    }

private:
    Hardware hw;
    controller_manager::ControllerManager cm;
    ros::Timer loop_timer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_hardware_interface");
    ros::NodeHandle n;
    Node node(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

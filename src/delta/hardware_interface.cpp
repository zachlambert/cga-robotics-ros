#include <cmath>
#include <vector>
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <stdio.h>

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

int get_fd()
{
    struct termios serial;

    char device[15] = "/dev/ttyACM0";

    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC);

    if (fd == -1) {
        std::cerr << "Failed to open serial device" << std::endl;
        return 1;
    }
    if (tcgetattr(fd, &serial) < 0) {
        std::cerr << "Failed to get configuration" << std::endl;
        return 1;
    }

    serial.c_iflag = 0;
    serial.c_oflag = 0;
    serial.c_lflag = 0;
    serial.c_cflag = 0;
    serial.c_cc[VMIN] = 0;
    serial.c_cc[VTIME] = 0;
    serial.c_cflag = B9600 | CS8 | CREAD;

    tcsetattr(fd, TCSANOW, &serial);

    return fd;
}

void write_pos(int fd, int i, double angle)
{
    double degrees = angle*180/M_PI + 45;
    if (degrees < 0) degrees = 0;
    if (degrees > 90) degrees = 90;
    static uint16_t signatures[] = {222, 333, 111, 444, 555};
    uint16_t cmd = (2500.0*10.0/513) * degrees;
    uint32_t packet = cmd << 16 | signatures[i];
    write(fd, &packet, 4);
}

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

        fd = get_fd();
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
            write_pos(fd, i, cmd[i]);
        }
    }

private:
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::PositionJointInterface cmd_interface;

    std::vector<std::string> names;
    std::vector<double> pos, vel, eff, cmd;

    int fd;
};

class Node {
public:
    Node(ros::NodeHandle& n): hw(n), cm(&hw, n)
    {
        loop_timer = n.createTimer(
            ros::Duration(1.0/25),
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

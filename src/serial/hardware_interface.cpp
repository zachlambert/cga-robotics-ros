#include <cmath>
#include <vector>

#include <libusb.h>
#include "protocol.h"

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

bool device_matches_vendor_product(libusb_device *device, unsigned short idVendor, unsigned short idProduct)
{
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    return idVendor == desc.idVendor && idProduct == desc.idProduct;
}

class Maestro {
    libusb_context *ctx;
    libusb_device **device_list;
    libusb_device_handle *device_handle;
    bool valid;

public:
    Maestro(): ctx(0), device_list(0), device_handle(0), valid(false)
    {
        valid = initialise();
    }
    bool initialise()
    {
        const unsigned short vendor_id = 0x1ffb;
        unsigned short product_id_array[]={0x0089, 0x008a, 0x008b, 0x008c};
        libusb_init(&ctx);
        int count=libusb_get_device_list(ctx, &device_list);
        for (int i=0; i<count; i++) {
            libusb_device *device = device_list[i];
            for (int id=0; id<4; id++) {
                if (device_matches_vendor_product(device, vendor_id, product_id_array[id])) {
                    libusb_open(device, &device_handle);
                    return true;
                }
            }
        }
        return false;
    }
    bool is_valid()const{ return valid; }

    ~Maestro()
    {
        libusb_close(device_handle);
        libusb_free_device_list(device_list, 0);
        libusb_exit(ctx);
    }

    void set_target(int servo, int command)
    {
        libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, command, servo, 0, 0, (ushort)5000);
    }

    void set_position(int servo, double pos)
    {
        // Position in degrees from -60 to 60
        int command = 4*(1472 + (pos/M_PI)*(2400-544));
        if (command < 4*800) command = 4*800;
        if (command > 4*2144) command = 4*2144;
        set_target(servo, command);
    }

    void disable(int servo)
    {
        set_target(servo, 0);
    }
};

struct Servo {
    int servo;
    double origin;
    double multiplier;
    Servo(): servo(-1), origin(0), multiplier(1) {}
    Servo(int servo): servo(servo), origin(0), multiplier(1) {}
    Servo(int servo, double origin): servo(servo), origin(origin), multiplier(1) {}
    Servo(int servo, double origin, double multiplier): servo(servo), origin(origin), multiplier(multiplier) {}
};

class Hardware: public hardware_interface::RobotHW {
public:
    Hardware(ros::NodeHandle &n): maestro()
    {
        names = {"theta_1", "theta_2", "theta_3", "theta_4",
            "theta_5", "theta_6", "gripper" };
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

        servos = {
            Servo(0), Servo(-1), Servo(3), Servo(4), Servo(5),
            Servo(6), Servo(7) };

        theta_1_servo_left = Servo(1);
        theta_1_servo_left = Servo(2, 0, -1);
    }

    ~Hardware()
    {
        for (Servo servo: servos) {
            if (servo.servo >= 0) maestro.disable(servo.servo);
        }
        maestro.disable(theta_1_servo_left.servo);
        maestro.disable(theta_1_servo_right.servo);
    }

    void read()
    {
        // For now, just assume pos is updated instantly.
        // The servo controller does actually have functionality to read
        // the current position. The cmd writes the target position, which
        // the servo moves towards with some max speed.
        pos = cmd;
    }

    void write()
    {
        if (!maestro.is_valid()) {
            ROS_ERROR("Couldn't connect to maestro");
        }
        for (std::size_t i = 0; i < servos.size(); i++) {
            int servo = servos[i].servo;
            if (servo < 0) continue;
            double pos = servos[i].origin + servos[i].multiplier*cmd[i];
            ROS_INFO("Pos = %f", pos);
            maestro.set_position(servo, pos);
        }

        double pos_1_left = theta_1_servo_left.origin + cmd[1]*theta_1_servo_left.multiplier;
        double pos_1_right = theta_1_servo_right.origin + cmd[1]*theta_1_servo_right.multiplier;
        maestro.set_position(theta_1_servo_left.servo, pos_1_left);
        maestro.set_position(theta_1_servo_right.servo, pos_1_right);
    }
private:
    Maestro maestro;
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::PositionJointInterface cmd_interface;

    std::vector<Servo> servos;
    std::vector<std::string> names;
    std::vector<double> pos, vel, eff, cmd;

    Servo theta_1_servo_left, theta_1_servo_right;
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
    ros::init(argc, argv, "serial_hardware_interface");
    ros::NodeHandle n;
    Node node(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

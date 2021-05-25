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

double alpha_to_beta(double alpha)
{
    ROS_INFO("Alpha: %f", alpha);
    static double a = 28, b = 54, c = 30.5, d = 60.5;
    double p_sq = b*b + c*c - 2*b*c*std::cos(alpha);
    double p = std::sqrt(p_sq);
    double beta1 = std::acos((p_sq+a*a-d*d)/(2*p*a));
    double beta2 = std::acos((p_sq+b*b-c*c)/(2*p*b));
    ROS_INFO("Beta1: %f", beta1);
    ROS_INFO("Beta2: %f", beta2);
    ROS_INFO("Beta: %f", beta1+beta2);
    return beta1 + beta2;
}

double transform_theta_3_angle(double angle)
{
    static double alpha0 = 98.0*M_PI/180.0;
    static double beta0 = alpha_to_beta(alpha0);
    ROS_INFO("Angle: %f", angle);
    double beta = alpha_to_beta(angle + alpha0);
    ROS_INFO("Command: %f", beta0-beta);
    return beta0 - beta;
}

struct Servo {
    int servo;
    double zero_pos; // Position of angle which corresponds to a zero command to the servo
    double multiplier;
    Servo(): servo(-1) {}
    Servo(int servo, double zero_pos): servo(servo), zero_pos(zero_pos), multiplier(1) {}
    Servo(int servo, double zero_pos, double multiplier): servo(servo), zero_pos(zero_pos), multiplier(multiplier) {}

    double get_pos(double angle)
    {
        angle = (angle - zero_pos);
        if (servo == 3) { // 0=theta_1, (1,2)=theta_2, 3=theta_3
            ROS_INFO("Input: %f", angle);
            angle = transform_theta_3_angle(angle);
            ROS_INFO("Output: %f", angle);
        }
        return multiplier*angle;
    }
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
            Servo(0, 0), Servo(), Servo(3, 100.0*(M_PI/180.0)), Servo(4, 0), Servo(5, 30.0*(M_PI/180), -1),
            Servo(6, 0), Servo(7, 35.0*(M_PI/180.0)) };

        theta_1_servo_left = Servo(1, 6.0*(M_PI/180.0), -1);
        theta_1_servo_right = Servo(2, 6.0*(M_PI/180.0), 1);
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
            double pos = servos[i].get_pos(cmd[i]);
            maestro.set_position(servo, pos);
        }

        maestro.set_position(theta_1_servo_left.servo, theta_1_servo_left.get_pos(cmd[1]));
        maestro.set_position(theta_1_servo_right.servo, theta_1_servo_right.get_pos(cmd[1]));
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

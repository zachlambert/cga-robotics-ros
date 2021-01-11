#include <ros/ros.h>

#include "cbot/delta.h"
namespace cbot { using namespace cga_impl; }

int main(int argc, char **argv)
{
    cbot::Delta::Structure structure;
    structure.base_radius = 0.2;
    cbot::Delta delta(structure);
    return 0;
}

#include <ros/ros.h>
#include "cbot/delta.h"
using namespace cga_impl;

int main(int argc, char **argv)
{
    Delta::Structure structure;
    structure.base_radius = 0.2;
    Delta delta(structure);
    return 0;
}

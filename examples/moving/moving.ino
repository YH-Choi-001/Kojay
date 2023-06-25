#include <Kojay.h>

void setup () {
    robot.begin();
    while (!robot.menu()) {}
}

void loop () {
    robot.polar_ctrl(0, 80, 0.98 * robot.get_rotation_spd(0));
}
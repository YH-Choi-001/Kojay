#include <Kojay.h>

void setup () {
    robot.begin();
    while (!robot.menu()) {}
    while (robot.get_uts_dist(front) > 5) {}
    while (robot.get_heading() > 7 && robot.get_heading() < 353) {
        robot.rect_ctrl(0, 0, 0.98 * robot.get_rotation_spd(0));
    }
}

void loop () {
    robot.move_to(back, 30, right, 60);
}
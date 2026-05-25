#include "car_main.hpp"

#include "car_control.hpp"

extern "C" void my_main(void)
{
    car::controller.init();

    for (;;) {
        car::controller.poll();
    }
}

#include <iostream>
#include "EV3RT.h"

class Driver
{
public:
    Driver(int32_t *bt_cmd);
    ~Driver();

private:
    EV3RT* robo;
};

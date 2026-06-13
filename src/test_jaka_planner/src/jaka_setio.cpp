#include <iostream>
#include "jaka_sdk/JAKAZuRobot.h"
#include <unistd.h>
// 设置与查询数字输出
int main()
{
    // tools IO
    BOOL DO1, DO2;
    BOOL state1 = 0, state2 = 0;
    // 实例API对象demo
    JAKAZuRobot demo;
    // 登录控制器，需要将192.168.2.194替换为自己控制器的IP
    demo.login_in("10.5.5.100");
    // 机械臂上电
    demo.power_on();
    // Main spin loop
    while (1)
    {
        // Check if DO1 and DO2 are on
        auto ret = demo.get_digital_output(IO_TOOL, 0, &DO1);
        if (ret == ERR_SUCC)
        {
            if (state1 != DO1)
            {
                state1 = DO1;
                printf("DO1: %d\n", DO1);
            }
        }
        ret = demo.get_digital_output(IO_TOOL, 1, &DO2);
        if (ret == ERR_SUCC)
        {
            if (state2 != DO2)
            {
                state2 = DO2;
                printf("DO2: %d\n", DO2);
            }
        }
    }
    return 0;
}
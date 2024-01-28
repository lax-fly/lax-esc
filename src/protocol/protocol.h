#pragma once

// 1 - 5：beep（1 = low freq.5 = high freq）
//     6：esc信息请求（fw版本和通过tlm线发送的SN）
//     7：一个方向旋转
//     8：另一个方向旋转
//     9：3d模式关闭
//     10：3d模式打开
//     11：esc设置请求（saved settings over the TLM wire）
//     12：保存设置

class Protocol
{
public:
    enum CMD
    {
        NONE,
        BEEP,
        VERSION,
        DIR,
        MODE_3D,
        SETTING,
        SAVE_SETTING,
        THROTTLE,
    };
    enum Type
    {
        DSHOT,
        SERIAL,
    };
    struct Package
    {
        CMD cmd;
        union
        {
            int value;
            const char *str;
        };
    };
    typedef void (*CallBack)(const Package &package);
    // the callback should copy the package(including the str), then process it outside the callback, warning: don't make the callback take too long
    virtual void set_package_callback(CallBack callback) = 0;
    virtual void poll(void) = 0;
    static Protocol *singleton(Type type);
};
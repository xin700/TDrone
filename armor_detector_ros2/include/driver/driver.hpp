#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>

namespace DRIVER
{

    namespace SerialReadData
    {
        struct RobotStatus
        {
            // int(真实血量 / 10)
            std::array<uint8_t, 7> enemy_health; //0:base 1:hero 2:engineer 3:infantry3 4:infantry4 5:infantry5 6:sentry 7:outpost\

            /*裁判系统v1.5
            0 1 机器人回血增益（百分比，值为 10 意为每秒回复 10%最大血量）
            1 1 机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
            2 1 机器人防御增益（百分比，值为 50 意为 50%防御增益）
            3 2 机器人攻击增益（百分比，值为 50 意为 50%攻击增益）*/

            // 0:回血增益 1:枪口冷却倍率 2:防御增益 3:攻击增益
            std::array<uint8_t, 4> robot_buff;

            // 0:枪口冷却 1:枪口热量上限 2:枪口当前热量 3:剩余允许发弹量
            std::array<uint8_t, 4> robot_gun;
            uint16_t crc16;
        } __attribute__((packed));

        struct IMU_Flag
        {
            int8_t start_flag;
            float pitch_now;
            float yaw_now;
            float roll_now;
            float actual_bullet_speed;
            uint8_t aim_request;
            uint8_t mode_want;
            uint8_t enemy_color; // 0:red 1:blue
            uint16_t crc16;
        } __attribute__((packed));
    }

    struct SerialOK
    {
        bool serial_ok;
    };

    struct SerialWriteData
    {
        int8_t start_flag;
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        uint16_t crc16;
    } __attribute__((packed));

    struct PicStamp
    {
        cv::Mat pic;
        long time_stamp;
    };

    struct ClockBeginTime
    {
        long begin_time;
    };

    enum class AimMode
    {
        AIM_ARMOR,
        AIM_LARGE_BUFF,
        AIM_SMALL_BUFF,
        SUSPEND
    };

}

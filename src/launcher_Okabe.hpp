#include "PID.hpp"

uint8_t DATA[8] = {0};
int DJI_ID = 0x200;
int16_t speed = 0;

const float kp = 0.1;
const float ki = 0.035;
const float kd = 0.0;
const float sample_time = 0.02; // 20ms sample time
PID pid_controller(kp, ki, kd, sample_time);

bool flag = false;

void stop_motor(int zero)
{
    for (int i = 0; i < 8; i += 2)
    {
        DATA[i] = (zero >> 8) & 0xFF; // 上位バイト
        DATA[i + 1] = zero & 0xFF;    // 下位バイト
    }
}

/// @brief
/// @param now_time チャタリング防止
/// @return flagの状態(trueにして500msに停止にする

bool fire_launcher(int now_time, int pre, int now)
{
    if (now_time > 1000)
    {
        printf("FIRE\n");
        int16_t target = 15000;
        float output = pid_controller.calculate(target, 0);
        int16_t signed_output = static_cast<int16_t>(-output);
        DATA[0] = (signed_output >> 8) & 0xFF; // 上位バイト
        DATA[1] = signed_output & 0xFF;        // 下位バイト
        signed_output = static_cast<int16_t>(output);
        DATA[2] = (signed_output >> 8) & 0xFF; // 上位バイト
        DATA[3] = signed_output & 0xFF;        // 下位バイト
        signed_output = static_cast<int16_t>(-output);
        DATA[4] = (signed_output >> 8) & 0xFF; // 上位バイト
        DATA[5] = signed_output & 0xFF;        // 下位バイト
        signed_output = static_cast<int16_t>(output);
        DATA[6] = (signed_output >> 8) & 0xFF; // 上位バイト
        DATA[7] = signed_output & 0xFF;        // 下位バイト

        pre = now;
        flag = true;
    }
    return pre, now, flag;
}

// canをどうやって送ってるのかわかんなかったので以下のやつをいい感じに追加してください
/*
if (now_1 - pre_1 > 30ms)
{
    CANMessage msg(DJI_ID, DATA, 8);
    if (can.write(msg))
    {
        can.reset();
        // CANコントローラをリセット
        //  printf("OK\n");
    }
    else
    {
        printf("Can't send Message\n");
        printf("CAN Bus Error Status: %d\n", can.rderror());
        printf("CAN Bus Write Error Count: %d\n", can.tderror());
        if (can.rderror() == 255 || can.tderror() == 249)
        {
            printf("Resetting CAN controller\n");
            can.reset();
        }
    }
    pre_1 = now_1;
}
*/
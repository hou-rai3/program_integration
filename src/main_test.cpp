/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F26Q84
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★追加しましたby岡部★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

#include "launcher_Okabe.hpp"
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
#include "mcc_generated_files/mcc.hpp"
#include <string.h>
#include <math.h>

#define SM 0.707106781

/*
                         Main application
 */
typedef struct
{
    // int16_t m_id;
    int16_t mech_angle;
    int16_t rot_speed;
    int16_t current;
    uint8_t temp;
} motor_info;

typedef struct
{ // 足回り構造体
    signed short motor[4];
    uint16_t m_id;
} ashimawari;

typedef struct
{
    unsigned char Select : 1;
    unsigned char L3 : 1;
    unsigned char R3 : 1;
    unsigned char Start : 1;
    unsigned char Up : 1;
    unsigned char Right : 1;
    unsigned char Down : 1;
    unsigned char Left : 1;
    unsigned char L2 : 1;
    unsigned char R2 : 1;
    unsigned char L1 : 1;
    unsigned char R1 : 1;
    unsigned char Triangle : 1;
    unsigned char Circle : 1;
    unsigned char Cross : 1;
    unsigned char Square : 1;

    signed short LX;
    signed short LY;
    signed short RX;
    signed short RY;
} PS2Con;

motor_info moin[8];

PS2Con pc;

CAN_MSG_OBJ cmo_tx = {0};
CAN_MSG_FIELD cmf = {0};
CAN_MSG_OBJ cmo_rx = {0};

unsigned char candata[8] = {0};
signed short motor[8] = {0};

#define Motor_14 0x200
#define Motor_58 0x1FF

#define PI23 2.094395102

void can_init()
{
    cmf.idType = 0;
    cmf.frameType = 0;
    cmf.dlc = 8;
    cmf.formatType = 0;
    cmf.brs = 0;
}

void MDcan_send(uint32_t id, signed short *m)
{
    cmo_tx.msgId = id;
    cmo_tx.field = cmf;
    memcpy(candata, m, 8);
    cmo_tx.data = candata;
    CAN1_Transmit(TXQ, &cmo_tx);
}

unsigned char revbit(unsigned char x)
{
    x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1);
    x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2);
    return (x << 4) | (x >> 4);
}

void BLmotor_move(uint16_t id, signed short m1, signed short m2, signed short m3, signed short m4)
{
    cmo_tx.msgId = id;
    cmo_tx.field = cmf;
    cmo_tx.data[0] = (m1 >> 8);
    cmo_tx.data[1] = (m1 & 0xff);
    cmo_tx.data[2] = (m2 >> 8);
    cmo_tx.data[3] = (m2 & 0xff);
    cmo_tx.data[4] = (m3 >> 8);
    cmo_tx.data[5] = (m3 & 0xff);
    cmo_tx.data[6] = (m4 >> 8);
    cmo_tx.data[7] = (m4 & 0xff);
    CAN1_Transmit(TXQ, &cmo_tx);
}

// PID関係
float Kp = 3.06; // 比例
float Ki = 2.11; // 積分
float Kd = 2.0;  // 微分

signed int e;             // 誤差
signed int olde[8] = {0}; // 以前の誤差
signed int de;            // 微分
signed int in;            // 積分
float T = 0.005;          // 時間(ms)

signed int pc_ret;

signed int PID_control(unsigned char num, signed int inrpm, signed int gotrpm)
{ // PID処理
    e = inrpm - gotrpm;
    de = (e - olde[num]) / T;
    in = in + (e + olde[num]) * T / 2;
    olde[num] = e;
    pc_ret = e * Kp; //+in*Ki+de*Kd;

    if (pc_ret > 8000)
        pc_ret = 8000;
    if (pc_ret < -8000)
        pc_ret = -8000;

    return pc_ret;
}

void c910_rec()
{
    if (CAN1_Receive(&cmo_rx))
    {

        if (cmo_rx.msgId == 0x201)
        {
            moin[0].mech_angle = cmo_rx.data[0] << 8 | cmo_rx.data[1];
            moin[0].rot_speed = cmo_rx.data[2] << 8 | cmo_rx.data[3];
            moin[0].current = cmo_rx.data[4] << 8 | cmo_rx.data[5];
            moin[0].temp = cmo_rx.data[6];
        }
        if (cmo_rx.msgId == 0x202)
        {
            moin[1].mech_angle = cmo_rx.data[0] << 8 | cmo_rx.data[1];
            moin[1].rot_speed = cmo_rx.data[2] << 8 | cmo_rx.data[3];
            moin[1].current = cmo_rx.data[4] << 8 | cmo_rx.data[5];
            moin[1].temp = cmo_rx.data[6];
        }
        if (cmo_rx.msgId == 0x203)
        {
            moin[2].mech_angle = cmo_rx.data[0] << 8 | cmo_rx.data[1];
            moin[2].rot_speed = cmo_rx.data[2] << 8 | cmo_rx.data[3];
            moin[2].current = cmo_rx.data[4] << 8 | cmo_rx.data[5];
            moin[2].temp = cmo_rx.data[6];
        }
        if (cmo_rx.msgId == 0x204)
        {
            moin[3].mech_angle = cmo_rx.data[0] << 8 | cmo_rx.data[1];
            moin[3].rot_speed = cmo_rx.data[2] << 8 | cmo_rx.data[3];
            moin[3].current = cmo_rx.data[4] << 8 | cmo_rx.data[5];
            moin[3].temp = cmo_rx.data[6];
        }

        // printf("angle=%d,speed=%d,current=%d,temp=%d\r\n",moin[0].mech_angle,moin[0].rot_speed,moin[0].current,moin[0].temp);
        //            printf("%03x %02x%02x%02x%02x%02x%02x%02x%02x\r\n",
        //                    cmo_rx.msgId,cmo_rx.data[0],cmo_rx.data[1],cmo_rx.data[2],cmo_rx.data[3]
        //                    ,cmo_rx.data[4],cmo_rx.data[5],cmo_rx.data[6],cmo_rx.data[7]);
    }
}

// コントローラー関係
unsigned char size = 0;
unsigned char vsRxBuffer[21];
unsigned char vsTxBuffer[21];
unsigned char j, i = 0;
void ps2_rec()
{
    switch (i)
    {
    case 0: // poll once
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x42;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0xff;
        vsTxBuffer[4] = 0xff;
        size = 5;
        i++;
        break;
    }
    case 1: // go into config mode
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x43;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0x1;
        vsTxBuffer[4] = 0x0;
        size = 5;
        i++;
        break;
    }
    case 2: // analog mode
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x44;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0x1;
        vsTxBuffer[4] = 0x3;
        vsTxBuffer[5] = 0x0;
        vsTxBuffer[6] = 0x0;
        vsTxBuffer[7] = 0x0;
        vsTxBuffer[8] = 0x0;
        size = 9;
        i++;
        break;
    }
    case 3: //
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x4d;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0x0;
        vsTxBuffer[4] = 0x1;
        vsTxBuffer[5] = 0xff;
        vsTxBuffer[6] = 0xff;
        vsTxBuffer[7] = 0xff;
        vsTxBuffer[8] = 0xff;
        size = 9;
        i++;
        break;
    }
    case 4:
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x4f;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0xff;
        vsTxBuffer[4] = 0xff;
        vsTxBuffer[5] = 0x3;
        vsTxBuffer[6] = 0x00;
        vsTxBuffer[7] = 0x00;
        vsTxBuffer[8] = 0x00;
        size = 9;
        i++;
        break;
    }
    case 5: // exit config mode
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x43;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0x0;
        vsTxBuffer[4] = 0x5a;
        vsTxBuffer[5] = 0x5a;
        vsTxBuffer[6] = 0x5a;
        vsTxBuffer[7] = 0x5a;
        vsTxBuffer[8] = 0x5a;
        size = 9;
        i++;
        break;
    }
    case 6:
    {
        vsTxBuffer[0] = 0x1;
        vsTxBuffer[1] = 0x42;
        vsTxBuffer[2] = 0x0;
        vsTxBuffer[3] = 0xff;
        vsTxBuffer[4] = 0xff;
        vsTxBuffer[5] = 0x0;
        vsTxBuffer[6] = 0x0;
        vsTxBuffer[7] = 0x0;
        vsTxBuffer[8] = 0x0;
        vsTxBuffer[9] = 0x0;
        vsTxBuffer[10] = 0x0;
        vsTxBuffer[11] = 0x0;
        vsTxBuffer[12] = 0x0;
        vsTxBuffer[13] = 0x0;
        vsTxBuffer[14] = 0x0;
        vsTxBuffer[15] = 0x0;
        vsTxBuffer[16] = 0x0;
        vsTxBuffer[17] = 0x0;
        vsTxBuffer[18] = 0x0;
        vsTxBuffer[19] = 0x0;
        vsTxBuffer[20] = 0x0;
        size = 21;
        i++;
        break;
    }
    default:
        break;
    }
    SEL_SetLow();
    for (j = 0; j < size; j++)
    {
        vsRxBuffer[j] = revbit(SPI1_ExchangeByte(revbit(vsTxBuffer[j])));
        // printf("%x,",vsRxBuffer[j]);
        DELAY_microseconds(10);
    }
    SEL_SetHigh();

    // printf("\r\n");

    vsRxBuffer[3] = ~vsRxBuffer[3];
    vsRxBuffer[4] = ~vsRxBuffer[4];
    memcpy(&pc, &vsRxBuffer[3], 2);
    pc.RX = vsRxBuffer[5] - 128;
    pc.RY = vsRxBuffer[6] - 127;
    pc.LX = vsRxBuffer[7] - 128;
    pc.LY = vsRxBuffer[8] - 127;
}

void CustomFIFO1Handler(void)
{
    CAN1_ReceiveFrom(FIFO1, &cmo_rx);
    unsigned char cf = cmo_rx.msgId - 0x201;
    moin[cf].mech_angle = cmo_rx.data[0] << 8 | cmo_rx.data[1];
    moin[cf].rot_speed = cmo_rx.data[2] << 8 | cmo_rx.data[3];
    moin[cf].current = cmo_rx.data[4] << 8 | cmo_rx.data[5];
    moin[cf].temp = cmo_rx.data[6];
}

void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    CAN1_SetFIFO1HalfFullHandler(&CustomFIFO1Handler);

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    // INTERRUPT_GlobalInterruptDisable();

    TMR1_StartTimer();

    SPI1_Open(SPI1_DEFAULT);

    can_init();

    SEL_SetHigh();

    bool b_c = 0;
    bool hassyaf = 0;
    char hass = 0;

    bool left = 0, right = 0;
    unsigned char pidset = 1;
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★追加しましたby岡部★★★★★★★★★★★★★★★★★★★
    auto pre = HighResClock::now();
    auto pre_1 = pre;
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

    while (1)
    {
        // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★追加しましたby岡部★★★★★★★★★★★★★★★★★★★
        auto now = HighResClock::now();
        auto now_1 = HighResClock::now();
        // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

        // Add your application code
        TMR1_WriteTimer(0);
        TMR1_Reload();

        ps2_rec();

        if (vsRxBuffer[1] == 0x79 && vsRxBuffer[2] == 0x5a)
        {

            //            float dis=hypot(pc.LX,pc.LY)/127;
            //            float rot=atan2(pc.LX,pc.LY);
            //            motor[0]=sin(rot)*dis*18000+pc.RX*31;
            //            motor[1]=sin(rot-PI23)*dis*18000+pc.RX*31;
            //            motor[2]=sin(rot+PI23)*dis*18000+pc.RX*31;

            unsigned char speed = 117;

            motor[0] = (pc.LX * SM + pc.LY * SM) * speed + pc.RX * 20;
            motor[1] = (pc.LX * (-SM) + pc.LY * SM) * speed + pc.RX * 20;
            motor[2] = (pc.LX * (-SM) - pc.LY * SM) * speed + pc.RX * 20;
            motor[3] = (pc.LX * SM - pc.LY * SM) * speed + pc.RX * 20;

            // printf("%f,%f,%d,%d,%d\r\n",dis,rot,motor[0],motor[1],motor[2]);
            // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★追加しましたby岡部★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
            if (pc.R1 ^ b_c)
            {
                int x_time = now - pre;
                fire_launcher(x_time, pre, now);
                pre = now;
            }
            if (flag == true && now - pre > 500ms)
            {
                stop_motor(0);
                flag = false;
            }
            // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
            if (pc.Circle ^ b_c)
            {
                b_c = pc.Circle;
                if (b_c)
                {
                    hassyaf = !hassyaf;
                }
            }
            if (hassyaf)
            {
                hass = 1;
                motor[6] = 4000;
            }
            else
            {
                hass = 0;
                motor[6] = 0;
            }

            if (pc.Triangle)
            {
                motor[7] = 6000;
            }
            else
            {
                if (pc.Cross)
                {
                    motor[7] = -6000;
                }
                else
                {
                    motor[7] = 0;
                }
            }

            //            if(pc.Left ^ left){
            //                left=pc.Left;
            //                if(left){
            //                    pidset--;
            //                    if(pidset==0)pidset=1;
            //                }
            //            }
            //            if(pc.Right ^ right){
            //                right=pc.Right;
            //                if(right){
            //                    pidset++;
            //                    if(pidset==5)pidset=4;
            //                }
            //            }
            //            if(pidset==1){
            //                if(pc.Up)Kp+=0.001;
            //                if(pc.Down)Kp-=0.001;
            //                printf("P=%f\r\n",Kp);
            //            }
            //            if(pidset==2){
            //                if(pc.Up)Ki+=0.001;
            //                if(pc.Down)Ki-=0.001;
            //                printf("I=%f\r\n",Ki);
            //            }
            //            if(pidset==3){
            //                if(pc.Up)Kd+=0.001;
            //                if(pc.Down)Kd-=0.001;
            //                printf("D=%f\r\n",Kd);
            //            }
        }
        else
        {
            motor[0] = 0;
            motor[1] = 0;
            motor[2] = 0;
            motor[3] = 0;
        };

        // printf("%x,%x,%d,%d,%d,%d\r\n",vsRxBuffer[3],vsRxBuffer[4],moin[0].rot_speed,moin[1].rot_speed,moin[2].rot_speed,moin[3].rot_speed);
        // printf("<%d,%d,%d,%d>\r\n",motor[0],motor[1],motor[2],motor[3]);
        // printf("%d\r\n",PID_control(0,motor[0],moin[0].rot_speed));

        MDcan_send(0, motor);

        BLmotor_move(Motor_14, PID_control(0, motor[0], moin[0].rot_speed),
                     PID_control(1, motor[1], moin[1].rot_speed),
                     PID_control(2, motor[2], moin[2].rot_speed),
                     PID_control(3, motor[3], moin[3].rot_speed));
        BLmotor_move(Motor_58, hass * -16000,
                     hass * 16000,
                     PID_control(6, motor[6], moin[6].rot_speed),
                     PID_control(7, motor[7], moin[7].rot_speed));

        while (!TMR1_HasOverflowOccured())
            ;
        PIR3bits.TMR1IF = 0;
    }
}
/**
 End of File
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/joystick.h>
#include <string>
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringSerial.h>

enum leg_num
{
    front_right,
    middle_right,
    rear_right,
    rear_left,
    middle_left,
    front_left,
};

class support_polygon
{
public:
    double long_diagonal = 44;
    double short_diagonal = 22 * sqrt(3.0);
    double side = 22;
};

enum mode
{
    auto_attitude_mode,
    manualMode,
};

class hexapod_body_state
{
public:
    const double side = 7.4, long_diagonal = 14.1, short_diagonal = 7.4 * sqrt(3.0);
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    double cog_height = 12.5;
    double ZMP_x, ZMP_y;
    int mode = manualMode;
    int mode_is_alrdy_changed;
};

class leg_state
{
public:
    const double coxa_length = 5.5, femur_length = 7.4, tibia_length = 8.0;
    int coxa_encoder_val, femur_encoder_val, tibia_encoder_val; //value that order to encoder between 0 - 5000
    int coxa_id, femur_id, tibia_id;
    double coxa_arg, femur_arg, tibia_arg; //argument of joint -π[rad]〜+π[rad]
    double dx = 0.0, dy = 0.0, dz = 0.0;
    double x, y, z;
    double dx_home = 0.0, dy_home = 0.0, dz_home = 0.0;
    double x_home = 0.0, y_home = 0.0, z_home = 0.0;
    double x_base, y_base, z_base;
    double trajectory_hight = 10.0, trajectory_width = 5.0;
    double trajectory_yaw = 0.0, trajectory_pitch = 0.0;
    double trajectory_velocity;
};

typedef struct
{
    uint16_t X;
    uint16_t Y;
    uint16_t A;
    uint16_t B;
    uint16_t LB;
    uint16_t LT;
    uint16_t RB;
    uint16_t RT;
    uint16_t start;
    uint16_t back;
    int16_t axes1_x;
    int16_t axes1_y;
    int16_t axes0_x;
    int16_t axes0_y;
} controler_state;

void write_controler_state(controler_state *controler, js_event event)
{
    switch (event.type)
    {
    case JS_EVENT_BUTTON:
        if (event.number == 1)
        {
            controler->A = event.value;
        }
        if (event.number == 2)
        {
            controler->B = event.value;
        }
        if (event.number == 0)
        {
            controler->X = event.value;
        }
        if (event.number == 3)
        {
            controler->Y = event.value;
        }
        if (event.number == 4)
        {
            controler->LB = event.value;
        }
        if (event.number == 5)
        {
            controler->RB = event.value;
        }
        if (event.number == 6)
        {
            controler->LT = event.value;
        }
        if (event.number == 7)
        {
            controler->RT = event.value;
        }
        if (event.number == 9)
        {
            controler->start = event.value;
        }
        if (event.number == 0)
        {
            controler->X = event.value;
        }
        if (event.number == 3)
        {
            controler->Y = event.value;
        }
        if (event.number == 8)
        {
            controler->back = event.value;
            // printf("bock %d", controler->back);
        }

        break;
    case JS_EVENT_AXIS:
        if (event.number == 0)
        {
            controler->axes0_x = event.value;
        }
        if (event.number == 1)
        {
            controler->axes0_y = -event.value;
        }
        if (event.number == 2)
        {
            controler->axes1_x = event.value;
        }
        if (event.number == 3)
        {
            controler->axes1_y = -event.value;
        }
        break;
    default:
        /* Ignore init events. */
        break;
    }
}

void set_joint_arg_by_inv_dynamics(leg_state tmp_leg[])
{
    double tmp_x, tmp_y, tmp_z;
    int j = front_left;
    for (int i = 0; i <= j; i++)
    {
        //座標軸を正面に向ける（進行方向＝y方向）
        if (i == front_left)
        {
            tmp_leg[i].x = tmp_leg[i].x * cos(M_PI/6) + tmp_leg[i].y*sin(M_PI/6);
            tmp_leg[i].x = tmp_leg[i].x * cos(M_PI/6) + tmp_leg[i].y*sin(M_PI/6);
        }
        tmp_leg[i].coxa_arg = atan2(tmp_leg[i].y, tmp_leg[i].x);

        tmp_x = tmp_leg[i].x * cos(tmp_leg[i].coxa_arg) + tmp_leg[i].y * sin(tmp_leg[i].coxa_arg) - tmp_leg[i].coxa_length;
        tmp_y = -tmp_leg[i].x * sin(tmp_leg[i].coxa_arg) + tmp_leg[i].y * cos(tmp_leg[i].coxa_arg);
        tmp_z = tmp_leg[i].z;

        tmp_leg[i].tibia_arg = -(M_PI - acos((pow(tmp_leg[i].femur_length, 2) + pow(tmp_leg[i].tibia_length, 2) - pow(tmp_x, 2) - pow(tmp_z, 2)) / (2 * tmp_leg[i].femur_length * tmp_leg[i].tibia_length)));
        tmp_leg[i].femur_arg = atan2(tmp_z, tmp_x) + acos((pow(tmp_leg[i].femur_length, 2) - pow(tmp_leg[i].tibia_length, 2) + pow(tmp_x, 2) + pow(tmp_z, 2)) / (2 * tmp_leg[i].femur_length * sqrt(pow(tmp_x, 2) + pow(tmp_z, 2))));
    }
}

//サーボを使うためのデバイスファイル
int fd_servo = serialOpen("/dev/serial0", 115200);

void pub_encoder_bal_to_all_servo(leg_state leg[])
{
    int i;
    int j;
    unsigned char servo_buff[4];

    // radから0から5000の分解能による指令値に変更
    // モータの正負の違いもここて調節し直す
    leg[front_right].femur_encoder_val = -1 * (leg[front_right].femur_arg) * (2500 / M_PI) + 2500;
    leg[front_right].tibia_encoder_val = leg[front_right].tibia_arg * (2500 / M_PI) + 2500;
    leg[front_right].coxa_encoder_val = leg[front_right].coxa_arg * (2500 / M_PI) + 2500;

    leg[front_left].coxa_encoder_val = leg[front_left].coxa_arg * (2500 / (0.75 * M_PI)) + 2500;
    leg[front_left].femur_encoder_val = leg[front_left].femur_arg * (2500 / (0.75 * M_PI)) + 2500;
    leg[front_left].tibia_encoder_val = -1 * (leg[front_left].tibia_arg) * (2500 / (0.75 * M_PI)) + 2500;

    leg[middle_right].femur_encoder_val = -1 * (leg[middle_right].femur_arg) * (2500 / M_PI) + 2500;
    leg[middle_right].tibia_encoder_val = leg[middle_right].tibia_arg * (2500 / M_PI) + 2500;
    leg[middle_right].coxa_encoder_val = leg[middle_right].coxa_arg * (2500 / M_PI) + 2500;

    leg[middle_left].femur_encoder_val = (leg[middle_left].femur_arg) * (2500 / M_PI) + 2500;
    leg[middle_left].tibia_encoder_val = -1 * (leg[middle_left].tibia_arg) * (2500 / M_PI) + 2500;
    leg[middle_left].coxa_encoder_val = leg[middle_left].coxa_arg * (2500 / M_PI) + 2500;

    leg[rear_right].femur_encoder_val = -1 * (leg[rear_right].femur_arg) * (2500 / M_PI) + 2500;
    leg[rear_right].tibia_encoder_val = leg[rear_right].tibia_arg * (2500 / M_PI) + 2500;
    leg[rear_right].coxa_encoder_val = leg[rear_right].coxa_arg * (2500 / M_PI) + 2500;

    leg[rear_left].femur_encoder_val = leg[rear_left].femur_arg * (2500 / M_PI) + 2500;
    leg[rear_left].tibia_encoder_val = -1 * (leg[rear_left].tibia_arg) * (2500 / M_PI) + 2500;
    leg[rear_left].coxa_encoder_val = leg[rear_left].coxa_arg * (2500 / M_PI) + 2500;

    for (i = 0; i <= front_left; i++)
    {

        //coxaの司令
        servo_buff[0] = 0xff;
        servo_buff[1] = leg[i].coxa_id;
        servo_buff[2] = leg[i].coxa_encoder_val >> 7;   //角度指令値の上位８ビット
        servo_buff[3] = leg[i].coxa_encoder_val & 0x7f; //角度指令値の下位８ビット

        for (j = 0; j <= 3; j++)
        {
            serialPutchar(fd_servo, servo_buff[j]);
        }

        //femurの司令
        servo_buff[0] = 0xff;
        servo_buff[1] = leg[i].femur_id;
        servo_buff[2] = leg[i].femur_encoder_val >> 7;   //角度指令値の上位８ビット
        servo_buff[3] = leg[i].femur_encoder_val & 0x7f; //角度指令値の下位８ビット
        for (j = 0; j <= 3; j++)
        {
            serialPutchar(fd_servo, servo_buff[j]);
        }

        //tibiaの司令
        servo_buff[0] = 0xff;
        servo_buff[1] = leg[i].tibia_id;
        servo_buff[2] = leg[i].tibia_encoder_val >> 7;   //角度指令値の上位８ビット
        servo_buff[3] = leg[i].tibia_encoder_val & 0x7f; //角度指令値の下位８ビット
        for (j = 0; j <= 3; j++)
        {
            serialPutchar(fd_servo, servo_buff[j]);
        }
    }
}

void def_servo_id(leg_state leg[])
{
    leg[front_left].coxa_id = 1;
    leg[front_left].femur_id = 2;
    leg[front_left].tibia_id = 3;

    leg[middle_left].coxa_id = 4;
    leg[middle_left].femur_id = 6;
    leg[middle_left].tibia_id = 5;

    leg[rear_left].coxa_id = 8;
    leg[rear_left].femur_id = 9;
    leg[rear_left].tibia_id = 10;

    leg[front_right].coxa_id = 13;
    leg[front_right].femur_id = 15;
    leg[front_right].tibia_id = 14;

    leg[middle_right].coxa_id = 10;
    leg[middle_right].femur_id = 11;
    leg[middle_right].tibia_id = 12;

    leg[rear_right].coxa_id = 17;
    leg[rear_right].femur_id = 18;
    leg[rear_right].tibia_id = 16;
}

int main()
{
    ////コントローラ関係
    int fd = open("/dev/input/js0", O_RDONLY);
    struct js_event event;
    controler_state *controler;
    controler = (controler_state *)malloc(sizeof(controler_state));

    /////ヘクサポッド全体の定義
    double count = 0.0;
    hexapod_body_state body_state[1];
    support_polygon support_hexagon[1];
    leg_state leg[6];

    //サーボ関係の初期化
    def_servo_id(leg);
    unsigned char servo_buff[4];
    int tmp_buff;

    /////////////////////////////////////
    /////メイン制御ループ///////////////////
    /////////////////////////////////////
    while (1)
    {
        //コントローラの値の読み込み
        read(fd, &event, sizeof(event));
        write_controler_state(controler, event);

        // tmp_buff = 2500;
        // //900から1900までは動作確認済み

        // servo_buff[0] = 0xff;
        // servo_buff[1] = 2;
        // servo_buff[2] = tmp_buff >> 7;   //角度指令値の上位８ビット
        // servo_buff[3] = tmp_buff & 0x7f; //角度指令値の下位８ビット

        // for (int i = 0; i < 4; i++)
        // {
        //     // serialPuts(fd_servo, servo_buff);
        //     serialPutchar(fd_servo, servo_buff[i]);
        // }

        // leg[front_left].coxa_arg = (135.0 / 180.0) * M_PI;
        // leg[front_left].femur_arg = (135.0 / 180.0) * M_PI;
        // leg[front_left].tibia_arg = (135.0 / 180.0) * M_PI;

        leg[front_left].x = 0;
        leg[front_left].y = 20.9;
        leg[front_left].z = 0;

        set_joint_arg_by_inv_dynamics(leg);
        pub_encoder_bal_to_all_servo(leg);

        printf("coxa = %d , femur = %d, tibia = %d \n", leg[front_left].coxa_encoder_val, leg[front_left].femur_encoder_val, leg[front_left].tibia_encoder_val);
        usleep(100);
    }
    return 0;
}
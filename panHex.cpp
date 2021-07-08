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

class hexapod_body_state
{
public:
    const double side = 8.0, long_diagonal = 16.0, short_diagonal = 8 * sqrt(3.0);
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    double cog_height = 12.5;
    double ZMP_x, ZMP_y;
    int mode = manualMode;
    int mode_is_alrdy_changed;
};

class leg_state
{
public:
    const double coxa_length = 4, femur_length = 10, tibia_length = 12.5;
    int32_t coxa_encoder_val, femur_encoder_val, tibia_encoder_val; //value that order to encoder between 0 - 4048
    double coxa_arg, femur_arg, tibia_arg;                          //argument of joint -π[rad]〜+π[rad]
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

int main()
{
    ////コントローラ関係
    int fd = open("/dev/input/js0", O_NONBLOCK);
    int fd = open("/dev/input/js0", O_RDONLY);
    struct js_event event;
    controler_state *controler;
    controler = (controler_state *)malloc(sizeof(controler_state));

    /////ヘクサポッド全体の定義
    double count = 0.0;
    hexapod_body_state body_state[1];
    support_polygon support_hexagon[1];
    leg_state leg[6];

    /////メイン制御ループ
    while (1)
    {
        //コントローラの値の読み込み
        read(fd, &event, sizeof(event));
        write_controler_state(controler, event);
        
    }
}

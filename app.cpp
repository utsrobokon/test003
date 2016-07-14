#include "app.h"

#include "Tracer.h"
#include "pid.h"

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE  40         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */

/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c3マクロ */
#define TAIL_ANGLE_STAND_UP  80 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */


/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void tail_control(signed int angle);

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */


/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

//static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
//static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */


using namespace ev3api;

Tracer tracer;


void tracer_cyc(intptr_t exinf) {
    act_tsk(TRACER_TASK);
}

void tracer_task(intptr_t exinf) {
//  if (ev3_button_is_pressed(LEFT_BUTTON)) {
//    wup_tsk(MAIN_TASK);  // 左ボタン押下でメインを起こす
//  } else {
//    tracer.run();  // ;走行
//  }
//  ext_tsk();
}

static float diff[2];
static float integral;


#include "BalancerCpp.h"        // <1>
Balancer balancer;              // <1>

void main_task(intptr_t unused) {

    signed char forward;      /* 前後進命令 */
//    signed char turn;         /* 旋回命令 */
    float turn;               /* 旋回命令 */
    signed char pwm_L, pwm_R; /* 左右モータPWM出力 */



    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    fprintf(bt, "main_task start\n");
//    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);
    
    
    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);


    /* スタート待機 */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if (bt_cmd == 1)
        {
            bt_cmd = 0;
            break; /* リモートスタート */
        }
        if (bt_cmd == 2)
        {
            ev3_led_set_color(LED_ORANGE);
        }
        if (bt_cmd == 3)
        {
            ev3_led_set_color(LED_GREEN);
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* タッチセンサが押された */
        }

        tslp_tsk(10); /* 10msecウェイト */
    }
    

    fprintf(bt, "start dash\n");
    
    // 滑らかスタートお試し
    int i;
    i = 0;
    for(i = 0; i < 15 ; i++)
    {
        tail_control(TAIL_ANGLE_STAND_UP + i); 
        tslp_tsk(5); // 5msecウェイト
    }
    
/*
    while(1)
    {
        tail_control(TAIL_ANGLE_DRIVE);
        tslp_tsk(4);
    }
*/

    // 走行モーターエンコーダーリセット
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    // ジャイロセンサーリセット
    ev3_gyro_sensor_reset(gyro_sensor);
    balancer.init(GYRO_OFFSET);

    // pid制御リセット
    pid Pid;
    
    while(1)
    {
        int32_t motor_ang_l, motor_ang_r;
        int gyro, volt;
        int16_t sensor_val;
        int16_t target_val;
        

        // バックボタンが押されたら終了
        if (ev3_button_is_pressed(BACK_BUTTON)) break;
        
        // タッチセンサーが押されたら終了
        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1) break;
    
        // バランス走行用角度に制御
        tail_control(TAIL_ANGLE_DRIVE);

        // 障害物を検知したら停止
        if (sonar_alert() == 1)
        {
            // 障害物を検知したら停止
            forward = turn = 0;
        }
        else
        {
            // 前進命令
            forward = 40;
        }

        sensor_val = ev3_color_sensor_get_reflect(color_sensor);
        target_val = 20;

        if(1){
            float p, i, d;
            p = 0;
            i = 0;
            d = 0;

            diff[0] = diff[1];
            diff[1] = sensor_val - target_val;  //偏差を取得
            integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

            p = KP * diff[1];
            i = KI * integral;
            d = KD * (diff[1] - diff[0]) / DELTA_T;

            if(p + i+ d < -100)
            {
                turn = -100;
            }
            else if(p + i + d > 100)
            {
                turn = 100;
            }
            else
            {
                turn = p + i + d;
            }
        }

//        turn = Pid.getTurnVal(sensor_val, target_val);
        fprintf(bt, "sensor_val:%d target_val:%d TurnVal:%f \n", sensor_val, target_val, turn);
         
        // 倒立振子制御API に渡すパラメータを取得する
        motor_ang_l = ev3_motor_get_counts(left_motor);
        motor_ang_r = ev3_motor_get_counts(right_motor);
        gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        volt = ev3_battery_voltage_mV();

        fprintf(bt, "motor_ang_l:%d motor_ang_r:%d gyro:%d volt:%d\n", motor_ang_l, motor_ang_r, gyro, volt);
        
        // 倒立振子制御APIを呼び出し、倒立走行するための
        // 左右モータ出力値を得る
        fprintf(bt, "balancer.setCommand: %d : %d\n", forward, turn);
        balancer.setCommand(forward, turn);   // <1>
        balancer.update(gyro, motor_ang_r, motor_ang_l, volt); // <2>

        pwm_L = balancer.getPwmRight();       // <3>
        pwm_R = balancer.getPwmLeft();        // <3>
        fprintf(bt, "balancer: pwm_L=%d : pwm_R=%d\n", pwm_L, pwm_R);

        // EV3ではモーター停止時のブレーキ設定が事前にできないため
        // 出力0時に、その都度設定する
        if (pwm_L == 0)
        {
             ev3_motor_stop(left_motor, true);
        }
        else
        {
            ev3_motor_set_power(left_motor, (int)pwm_L);
        }

        if (pwm_R == 0)
        {
             ev3_motor_stop(right_motor, true);
        }
        else
        {
            ev3_motor_set_power(right_motor, (int)pwm_R);
        }

        // 4msec周期起動
        tslp_tsk(4);
    }

    // モーター停止
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    // BlueToothタスクを停止
    ter_tsk(BT_TASK);
    fclose(bt);
     
    // メインタスクを終了
    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{
    float pwm = (float)(angle - ev3_motor_get_counts(tail_motor))*P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }   
        fputc(c, bt); /* エコーバック */
    }
}


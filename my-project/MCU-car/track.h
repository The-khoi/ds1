/**
 * 文件: tracking_control.h
 * 功能: 循迹控制模块头文件
 */

#ifndef __TRACKING_CONTROL_H__
#define __TRACKING_CONTROL_H__

#include "MSPM0G3507.h"  // TI提供的MSPM0G3507头文件

// 停止线检测状态机枚举
typedef enum {
    STOP_LINE_DETECTING,    // 检测停止线
    STOP_LINE_TURNING_RIGHT, // 右转
    STOP_LINE_GOING_STRAIGHT, // 直行
    STOP_LINE_COMPLETED     // 完成
} StopLineState;

// 函数声明
void gpio_input_check_channel_5(void);
void init_motor_pwm(void);
void system_init(void);

// 电机控制函数
void set_motor_speeds(uint16_t left_speed, uint16_t right_speed);

// 获取编码器脉冲数
uint32_t get_encoder_ticks(uint8_t motor_id);

// 获取系统状态
uint16_t get_sensor_state(void);
StopLineState get_stop_line_state(void);

#endif // __TRACKING_CONTROL_H__


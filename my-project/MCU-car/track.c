/**
 * 文件: tracking_control.c
 * 功能: 基于5路红外传感器的循迹控制（MSPM0G3507在Keil中的适配）
 */

#include "tracking_control.h"
#include "MSPM0G3507.h"  // TI提供的头文件

// 定义电机控制相关参数
#define MOTOR_MAX_SPEED 1000    // 电机最大速度
#define MOTOR_MIN_SPEED 300     // 电机最小速度
#define TURN_SPEED_DIFFERENCE 200 // 转弯时的速度差
#define STOP_LINE_TURN_TIME 50   // 右转时间(ms)
#define STOP_LINE_STRAIGHT_TIME 5000 // 直行时间(ms)
#define WHEEL_CIRCUMFERENCE 20.0f // 轮子周长(cm)
#define ENCODER_TICKS_PER_ROTATION 360 // 编码器每转脉冲数

// 传感器状态结构体
typedef struct {
    uint8_t gray_bit[5];  // 5路传感器值
    uint16_t state;       // 组合后的状态值
} GraySensorState;

// 系统状态结构体
typedef struct {
    GraySensorState gray_state;
    int8_t gray_status[1];
    int8_t gray_status_backup[1][20];
    uint8_t flag_gray;
    uint16_t gray_status_worse;
} TrackingSystemState;

// 停止线检测状态机
typedef enum {
    STOP_LINE_DETECTING,    // 检测停止线
    STOP_LINE_TURNING_RIGHT, // 右转
    STOP_LINE_GOING_STRAIGHT, // 直行
    STOP_LINE_COMPLETED     // 完成
} StopLineState;

// 全局变量
static TrackingSystemState system_state;
static StopLineState stopLineState = STOP_LINE_DETECTING;
static uint32_t stopLineStartTime = 0;
static uint32_t leftEncoderStart = 0;
static uint32_t rightEncoderStart = 0;
static uint16_t stopLinePatternCounter = 0;

/**
 * 函数: read_sensors
 * 功能: 读取5路红外传感器的值
 */
static void read_sensors(void)
{
    // 读取GPIO端口值 (根据实际引脚分配修改)
    system_state.gray_state.gray_bit[0] = (P1->IN & BIT0) ? 1 : 0; // 假设连接到P1.0
    system_state.gray_state.gray_bit[1] = (P1->IN & BIT1) ? 1 : 0; // 假设连接到P1.1
    system_state.gray_state.gray_bit[2] = (P1->IN & BIT2) ? 1 : 0; // 假设连接到P1.2
    system_state.gray_state.gray_bit[3] = (P1->IN & BIT3) ? 1 : 0; // 假设连接到P1.3
    system_state.gray_state.gray_bit[4] = (P1->IN & BIT4) ? 1 : 0; // 假设连接到P1.4
    
    // 组合传感器值为一个状态字节
    system_state.gray_state.state = 0x0000;    
    for(uint16_t i=0; i<5; i++)
    {
        system_state.gray_state.state |= system_state.gray_state.gray_bit[i] << i;
    }
}

/**
 * 函数: update_history
 * 功能: 更新历史状态记录
 */
static void update_history(void)
{
    // 保存历史状态
    for(uint16_t i=19; i>0; i--)
    {
        system_state.gray_status_backup[0][i] = system_state.gray_status_backup[0][i-1];
    }
    system_state.gray_status_backup[0][0] = system_state.gray_status[0];
}

/**
 * 函数: line_tracking_control
 * 功能: 根据传感器状态实现循迹控制
 */
static void line_tracking_control(void)
{
    // 根据当前状态设置循迹状态值
    if(system_state.gray_state.state == 0x0000)
        system_state.flag_gray = 0;  // 全部检测到白线
    else
        system_state.flag_gray = 1;  // 至少有一个传感器检测到黑线
    
    // 仅在检测停止线状态下执行循迹逻辑
    if(stopLineState == STOP_LINE_DETECTING) {
        switch(system_state.gray_state.state)
        {
            case 0x000E:  // 01110 - 中间三个传感器检测到黑线，直行
            case 0x0004:  // 00100 - 仅中间传感器检测到黑线，直行
                set_motor_speeds(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
                break;
                
            case 0x000C:  // 01100 - 左侧两个传感器检测到黑线，向左转
            case 0x001C:  // 11100 - 左侧三个传感器检测到黑线，向左转
                set_motor_speeds(MOTOR_MAX_SPEED - TURN_SPEED_DIFFERENCE, MOTOR_MAX_SPEED);
                break;
                
            case 0x0006:  // 00110 - 右侧两个传感器检测到黑线，向右转
            case 0x0007:  // 00111 - 右侧三个传感器检测到黑线，向右转
                set_motor_speeds(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED - TURN_SPEED_DIFFERENCE);
                break;
                
            default:  // 其他状态（如边缘情况）
                // 保持上一次状态或添加特殊处理
                break;
        }
    }
}

/**
 * 函数: stop_line_detection
 * 功能: 检测停止线并执行相应动作
 */
static void stop_line_detection(void)
{
    switch(stopLineState) {
        case STOP_LINE_DETECTING:
            if(system_state.gray_state.state == 0x001F) {  // 11111 - 全部检测到黑线
                // 记录开始时间和编码器值
                stopLineStartTime = TIMER32_0->VALUE;
                leftEncoderStart = get_encoder_ticks(0);
                rightEncoderStart = get_encoder_ticks(1);
                
                // 右转几毫秒
                set_motor_speeds(MOTOR_MAX_SPEED - TURN_SPEED_DIFFERENCE, MOTOR_MAX_SPEED);
                stopLineState = STOP_LINE_TURNING_RIGHT;
            }
            break;
            
        case STOP_LINE_TURNING_RIGHT:
            // 右转一段时间
            if(TIMER32_0->VALUE - stopLineStartTime >= STOP_LINE_TURN_TIME) {
                // 切换到直行状态
                set_motor_speeds(MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
                stopLineState = STOP_LINE_GOING_STRAIGHT;
            }
            break;
            
        case STOP_LINE_GOING_STRAIGHT:
            // 计算行驶距离
            uint32_t leftTicks = get_encoder_ticks(0) - leftEncoderStart;
            uint32_t rightTicks = get_encoder_ticks(1) - rightEncoderStart;
            float leftDistance = (float)leftTicks * WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_ROTATION;
            float rightDistance = (float)rightTicks * WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_ROTATION;
            float averageDistance = (leftDistance + rightDistance) / 2.0f;
            
            // 如果行驶距离达到114cm或时间超时
            if(averageDistance >= 114.0f || TIMER32_0->VALUE - stopLineStartTime >= STOP_LINE_STRAIGHT_TIME) {
                // 完成停止线逻辑
                set_motor_speeds(0, 0);
                stopLineState = STOP_LINE_COMPLETED;
            }
            break;
            
        case STOP_LINE_COMPLETED:
            // 停止线逻辑已完成，可以添加其他操作
            break;
    }
}

/**
 * 函数: special_pattern_detection
 * 功能: 检测特殊模式（如原代码中的0x00F0等模式）
 */
static void special_pattern_detection(void)
{
    switch(system_state.gray_state.state)
    {
        case 0x0030: stopLinePatternCounter++; break; // 000000110000b
        case 0x0020: stopLinePatternCounter++; break; // 000000100000b
        case 0x0060: stopLinePatternCounter++; break; // 000001100000b
        case 0x0040: stopLinePatternCounter++; break; // 000001000000b
        case 0x00C0: stopLinePatternCounter++; break; // 000011000000b
        case 0x00F0: // 000011110000b
        {
            if(stopLinePatternCounter >= 10)
            {
                stopLinePatternCounter = 0;
                // 触发蜂鸣器或其他动作
                P2->OUT |= BIT0;  // 假设蜂鸣器连接到P2.0
                TIMER32_1->LOAD = 3000000; // 3秒
                TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
            }
        }
        break;
        default:
            stopLinePatternCounter = 0; // 重置计数器
            break;
    }
}

/**
 * 函数: gpio_input_check_channel_5
 * 功能: 主控制函数，整合所有功能模块
 */
void gpio_input_check_channel_5(void)
{
    read_sensors();
    update_history();
    line_tracking_control();
    stop_line_detection();
    special_pattern_detection();
}


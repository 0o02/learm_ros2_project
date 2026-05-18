/**
  ******************************************************************************
  * @file    ros2_communication.c
  * @brief   ROS2通信模块源文件（二进制协议版本）
  *         实现二进制帧接收、解析和命令处理
  *         帧格式：AA 55 CMD LEN DATA[N] CHECK
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ros2_communication.h"
#include "usart2.h"
#include "pwm_servo_control.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static RxState rx_state = STATE_WAIT_HEADER1;
static BinaryFrame rx_frame;
static uint8_t data_index = 0;
static uint8_t calc_check = 0;

/* 发送单个字节 */
static void send_byte(uint8_t b)
{
    USART2_SendChar(b);
}

/**
  * @brief  发送二进制应答帧
  * @param  status: 状态码（0x00=成功，0x01=执行中，0xFE=错误）
  * @param  data: 数据段指针
  * @param  len: 数据段长度
  * @retval None
  *
  * 帧格式：BB 55 STATUS LEN DATA[0..LEN-1] CHECK
  * CHECK = BB ^ 55 ^ STATUS ^ LEN ^ DATA[0] ^ ... ^ DATA[LEN-1]
  */
void ros2_communication_send_response(uint8_t status, uint8_t *data, uint8_t len)
{
    uint8_t check = RESP_HEADER1 ^ RESP_HEADER2 ^ status ^ len;
    
    send_byte(RESP_HEADER1);
    send_byte(RESP_HEADER2);
    send_byte(status);
    send_byte(len);
    
    for (uint8_t i = 0; i < len; i++) {
        send_byte(data[i]);
        check ^= data[i];
    }
    
    send_byte(check);
}

/**
  * @brief  复位解析状态机
  * @retval None
  */
static void reset_parser(void)
{
    rx_state = STATE_WAIT_HEADER1;
    data_index = 0;
    calc_check = 0;
}

/**
  * @brief  每收到一个字节调用该函数
  * @param  byte: 接收到的字节
  * @retval None
  *
  * 状态机解析流程：
  * WAIT_HEADER1 -> WAIT_HEADER2 -> WAIT_CMD -> WAIT_LEN -> WAIT_DATA -> WAIT_CHECK
  * 校验通过后调用 process_binary_command() 处理命令
  */
void ros2_communication_process_data(uint8_t byte)
{
    switch (rx_state) {
        case STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER1) {
                rx_state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER2) {
                rx_state = STATE_WAIT_CMD;
                calc_check = FRAME_HEADER1 ^ FRAME_HEADER2;
            } else {
                reset_parser();
            }
            break;

        case STATE_WAIT_CMD:
            rx_frame.cmd = byte;
            calc_check ^= byte;
            rx_state = STATE_WAIT_LEN;
            break;

        case STATE_WAIT_LEN:
            rx_frame.data_len = byte;
            calc_check ^= byte;
            if (byte == 0) {
                rx_state = STATE_WAIT_CHECK;
            } else if (byte <= sizeof(rx_frame.data)) {
                data_index = 0;
                rx_state = STATE_WAIT_DATA;
            } else {
                reset_parser();
            }
            break;

        case STATE_WAIT_DATA:
            rx_frame.data[data_index++] = byte;
            calc_check ^= byte;
            if (data_index >= rx_frame.data_len) {
                rx_state = STATE_WAIT_CHECK;
            }
            break;

		case STATE_WAIT_CHECK:
			rx_frame.check = byte;
			if (calc_check == byte) {
				process_binary_command(&rx_frame);
			}
			reset_parser();
			break;

        default:
            reset_parser();
            break;
    }
}

/**
  * @brief  处理解析完成的二进制命令
  * @param  frame: 解析后的二进制帧指针
  * @retval None
  *
  * 当前支持的命令：
  * - CMD_MOVE_TO (0x01): 执行多关节运动
  * - CMD_STOP (0x02): 停止运动（预留）
  * - CMD_INIT (0x03): 初始化舵机
  *
  * MOVE_TO 数据格式（13字节）：
  *   [0]     point_index (uint8, 轨迹点索引，可忽略)
  *   [1-2]   joint1 PWM (uint16 LE, 500~2500)
  *   [3-4]   joint2 PWM (uint16 LE)
  *   [5-6]   joint3 PWM (uint16 LE)
  *   [7-8]   joint4 PWM (uint16 LE)
  *   [9-10]  joint5 PWM (uint16 LE)
  *   [11-12] time_ms (uint16 LE, 运动时间)
  */
void process_binary_command(BinaryFrame *frame)
{
	USART2_SendChar(0xCC);
    if (frame->cmd == CMD_MOVE_TO) {
        if (frame->data_len < MOVE_TO_DATA_LEN) return;
        
        // 解析 point_index（可忽略）
        // uint8_t point_idx = frame->data[0];
        
        // 解析5个关节PWM值（小端格式）
        uint16_t pwm[5];
        for (int i = 0; i < JOINT_COUNT; i++) {
            pwm[i] = frame->data[1 + i * 2] | (frame->data[2 + i * 2] << 8);
            
            // 限幅处理
            if (pwm[i] < 500) pwm[i] = 500;
            if (pwm[i] > 2500) pwm[i] = 2500;
        }
        
        // 解析运动时间（毫秒，小端格式）
        uint16_t time_ms = frame->data[11] | (frame->data[12] << 8);
        
        // 转换为float数组供ExecuteMultiServoCommand使用
        float pwm_float[5];
        for (int i = 0; i < JOINT_COUNT; i++) {
            pwm_float[i] = (float)pwm[i];
        }
        
        // 执行多关节运动
        ExecuteMultiServoCommand(pwm_float, (uint32_t)time_ms);
        
        // 发送"executing"应答，包含目标位置
        uint8_t resp_data[10];
        for (int i = 0; i < JOINT_COUNT; i++) {
            resp_data[i * 2]     = (uint8_t)(pwm[i] & 0xFF);
            resp_data[i * 2 + 1] = (uint8_t)((pwm[i] >> 8) & 0xFF);
        }
        ros2_communication_send_response(STATUS_EXECUTING, resp_data, 10);
        
    } else if (frame->cmd == CMD_STOP) {
        // 停止逻辑（预留）
        ros2_communication_send_response(STATUS_SUCCESS, NULL, 0);
        
    } else if (frame->cmd == CMD_INIT) {
        // 初始化：所有关节回到中位（1500）
        float init_pwm[5] = {1500.0f, 1500.0f, 1500.0f, 1500.0f, 1500.0f};
        ExecuteMultiServoCommand(init_pwm, 1000);
        ros2_communication_send_response(STATUS_SUCCESS, NULL, 0);
    }
    // 未知命令：静默丢弃
}

/**
  * @brief  初始化ROS2通信模块
  * @retval None
  *
  * 初始化解析状态机，发送二进制握手帧
  */
void ros2_communication_init(void)
{
    reset_parser();
    
    // 发送二进制握手帧（状态=成功，无数据）
    ros2_communication_send_response(STATUS_SUCCESS, NULL, 0);
}

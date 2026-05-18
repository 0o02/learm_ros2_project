/**
  ******************************************************************************
  * @file    ros2_communication.h
  * @brief   ROS2通信模块头文件（二进制协议版本）
  *         使用二进制帧替代JSON格式，提高通信效率
  ******************************************************************************
  */

#ifndef __ROS2_COMMUNICATION_H
#define __ROS2_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* 命令类型 */
#define CMD_MOVE_TO         0x01
#define CMD_STOP            0x02
#define CMD_INIT            0x03

/* 状态码 */
#define STATUS_SUCCESS      0x00
#define STATUS_EXECUTING    0x01
#define STATUS_ERROR        0xFE

/* 帧常量 */
#define RX_BUF_SIZE         64
#define FRAME_HEADER1       0xAA
#define FRAME_HEADER2       0x55
#define RESP_HEADER1        0xBB
#define RESP_HEADER2        0x55

/* 关节数 */
#define JOINT_COUNT         5

/* MOVE_TO命令数据长度 */
#define MOVE_TO_DATA_LEN    13  /* 1(point_index) + 5*2(pwm) + 2(time_ms) */

/* 解析状态 */
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECK
} RxState;

/* 二进制帧结构 */
typedef struct {
    uint8_t cmd;                    // 命令类型
    uint8_t data_len;               // 数据长度
    uint8_t data[32];               // 数据段
    uint8_t check;                  // 校验和
} BinaryFrame;

/* 函数声明 */
void ros2_communication_init(void);
void ros2_communication_process_data(uint8_t byte);
void ros2_communication_send_response(uint8_t status, uint8_t *data, uint8_t len);
void process_binary_command(BinaryFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* __ROS2_COMMUNICATION_H */

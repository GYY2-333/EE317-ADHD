#ifndef APP_HID_HANDLER_H
#define APP_HID_HANDLER_H

#include <stdint.h>

/* ========== 全局变量（extern 声明） ========== */
extern volatile uint8_t hid_cmd_received;
extern uint8_t hid_out_buffer[64];
extern uint8_t hid_in_buffer[64];
extern const uint8_t handshake_response[64];          /* 阻抗检测握手应答包 */
extern const uint8_t signal_handshake_response[64];   /* 信号检测握手应答包 */

/* ========== 状态枚举 ========== */
typedef enum {
    STATE_WAIT_HANDSHAKE = 0,   /* 等待主机发送握手包 */
    STATE_SEND_RESPONSE  = 1,   /* 已收到握手包，即将/已发送应答 */
    STATE_STREAMING      = 2    /* 流模式：ADC数据持续发送 */
} CommState;

/* ========== 工作模式枚举 ========== */
typedef enum {
    MODE_IMPEDANCE = 0,  /* 阻抗检测模式（0xF8 握手） */
    MODE_SIGNAL    = 1   /* 信号检测模式（0x80 握手） */
} WorkMode;

/* ========== 环形 ADC 采样缓冲区 ========== */
#define ADC_RING_SIZE 30
extern volatile uint16_t adc_ring[ADC_RING_SIZE];
extern volatile uint8_t  adc_ring_idx;

/* ========== 通信状态 ========== */
extern volatile CommState comm_state;
extern volatile WorkMode current_mode;

/* ========== 定时器标志 ========== */
extern volatile uint8_t timer_fired;   /* TIM3 帧发送触发 */

/* ========== 对外 API ========== */
void USB_FlushADCRingBuffer(void);
void APP_DeviceCustomHIDProcessReceivedReport(const uint8_t *report, uint16_t report_len);
void USB_BuildAndSendDataFrame(void);

#endif /* APP_HID_HANDLER_H */

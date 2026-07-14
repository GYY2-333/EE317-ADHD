#include "app_hid_handler.h"
#include "main.h"
#include "usbd_customhid.h"
#include <stdint.h>
#include <string.h>

/* ========== 通信状态（全局定义） ========== */
volatile CommState comm_state = STATE_WAIT_HANDSHAKE;
volatile WorkMode current_mode = MODE_SIGNAL;  /* 默认信号检测模式 */

/* ========== HID 缓冲区 ========== */
uint8_t hid_in_buffer[64];
uint8_t hid_out_buffer[64];
volatile uint8_t hid_cmd_received = 0;

/* ========== 线性 ADC 采样缓冲区 ========== */
volatile uint16_t adc_ring[ADC_RING_SIZE] = {0};
volatile uint8_t  adc_ring_idx  = 0;

/* ========== 定时器标志 ========== */
volatile uint8_t timer_fired  = 0;

/* ========== 帧序号、类型ID、tick 计数器 ========== */
static uint8_t  frame_seq   = 0;       /* Byte0: 帧序号 0~255 循环 */
static uint8_t  type_id     = 0x80;    /* Byte1: 类型 ID, 0x80~0x8F 循环 */
static uint16_t tick_count  = 0;       /* Byte2-3: tick 计数 */

#define SAMPLES_PER_FRAME  30    /* Byte4~63: 30 采样点 × 2 字节 */

/* ========== 两种握手应答包 ========== */

/* 阻抗检测握手应答包：主机先发 0xF8 再发 0x80 后，设备以此包回复 */
const uint8_t handshake_response[64] = {
    0x77, 0x62, 0x11, 0x00, 0x57, 0xD3, 0x1F, 0x25,
    0x00, 0x00, 0x00, 0x00, 0x96, 0x35, 0xFB, 0x67,
    0x21, 0x70, 0xC4, 0x35, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0x59, 0x86, 0x69, 0x84,
    0x35, 0x8F, 0x44, 0xA9, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x0D, 0xC7, 0x0B, 0xC5, 0xB8, 0xA7, 0x9A, 0x02,
};

/* 信号检测握手应答包：对主机直接发 0x80 后的回复 */
const uint8_t signal_handshake_response[64] = {
    0x77, 0x62, 0x11, 0x00, 0x4B, 0xD4, 0x9D, 0x27,
    0x00, 0x00, 0x00, 0x00, 0x86, 0xE6, 0x7E, 0xEA,
    0x51, 0xED, 0x48, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xB9, 0x85, 0x64, 0x86,
    0x84, 0x8F, 0xF1, 0xA5, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xC0, 0x19, 0xBE, 0xAC, 0xC9, 0x6C, 0x00,
};

/* ========== 帧计数器推进（信号检测模式：tick 步进 +10） ========== */
static void AdvanceFrameCounters(void)
{
    /* Byte 0: 帧序号 0~255 循环，满 0xFF 时 byte1 进位 */
    uint8_t prev_frame_seq = frame_seq;
    frame_seq++;
    if(frame_seq < prev_frame_seq)   /* 溢出：255 -> 0 */
    {
        /* Byte 1: 类型 ID 仅当帧序号溢出时进位，0x80~0x8F 循环 */
        type_id++;
        if(type_id > 0x8F)
            type_id = 0x80;
    }

    /* Byte 2-3: tick 计数, 信号检测每次 +10 */
    uint8_t tick_low  = (uint8_t)(tick_count & 0xFF);
    uint8_t tick_high = (uint8_t)(tick_count >> 8);
    uint8_t tick_step = 10;

    tick_low += tick_step;
    if(tick_low < tick_step)   /* 加法溢出 */
    {
        tick_high++;
        if(tick_high > 0x07)
        {
            tick_high = 0;
            tick_low  = 0;
        }
    }
    tick_count = ((uint16_t)tick_high << 8) | tick_low;
}

/* ========== 清除缓冲区（流开始时调用） ========== */
void USB_FlushADCRingBuffer(void)
{
    uint8_t i;
    for(i = 0; i < ADC_RING_SIZE; i++)
        adc_ring[i] = 0;
    adc_ring_idx = 0;
}

/* ========== 数据帧构建与发送 ========== */
void USB_BuildAndSendDataFrame(void)
{

    /* Byte 0: 帧序号 */
    hid_in_buffer[0] = frame_seq;

    /* Byte 1: 类型 ID */
    hid_in_buffer[1] = type_id;

    /* Byte 2-3: tick 计数（小端） */
    hid_in_buffer[2] = (uint8_t)(tick_count & 0xFF);
    hid_in_buffer[3] = (uint8_t)(tick_count >> 8);

    /* Byte 4-63: 30 采样点 × 2 字节小端，直接内存拷贝 */
    memcpy(&hid_in_buffer[4], (const void *)adc_ring, SAMPLES_PER_FRAME * 2);

    /* 通过 USB CustomHID 发送
     * 不检查返回值：发送失败意味着总线异常（断连/复位），
     * USB 栈会触发 EVENT_RESET 重置 comm_state；
     * 重试无意义——adc_ring 已被 TIM2 ISR 持续覆盖，
     * 重发的是新旧混合的脏数据。上层通过 frame_seq 跳号感知丢帧。 */
    extern USBD_HandleTypeDef hUsbDeviceFS;
    (void)USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, hid_in_buffer, 64);

    /* 推进计数器（无论发送成功与否，帧数据已构建完毕） */
    AdvanceFrameCounters();
}

/* ========== 握手协议应答发送 ========== */
static void USB_SendHandshakeResponse(void)
{
    const uint8_t *resp = signal_handshake_response;  /* 信号检测固定用此包 */
    memcpy(hid_in_buffer, resp, 64);

    extern USBD_HandleTypeDef hUsbDeviceFS;
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, hid_in_buffer, 64);
}

/* ========== 接收数据处理：握手协议状态机 ========== */
void APP_DeviceCustomHIDProcessReceivedReport(const uint8_t *report, uint16_t report_len)
{
    if(report == NULL || report_len < 1)
        return;

    uint8_t cmd = report[0];

    /*
     * 信号检测握手流程：
     *   1. 主机发 0x80 → 设备回 signal_handshake_response (77 62...4B D4...)
     *   2. 主机发 0x8F → 进入流模式
     */

    switch(comm_state)
    {
        case STATE_WAIT_HANDSHAKE:
            if(cmd == 0x80)
            {
                USB_SendHandshakeResponse();
                comm_state = STATE_SEND_RESPONSE;
            }
            break;

        case STATE_SEND_RESPONSE:
            if(cmd == 0x80)
            {
                /* 握手重试：主机再次发 0x80，重新发送应答包 */
                USB_SendHandshakeResponse();
            }
            else if(cmd == 0x8F)
            {
                /* 握手完成，进入流模式 */
                comm_state = STATE_STREAMING;

                USB_FlushADCRingBuffer();
                frame_seq  = 0;
                type_id    = 0x80;
                tick_count = 0;
            }
            break;

        case STATE_STREAMING:
            if(cmd == 0x80)
            {
                /* 流模式中重握手：重新发送应答，回到等待应答状态 */
                USB_SendHandshakeResponse();
                comm_state = STATE_SEND_RESPONSE;
            }
            break;

        default:
            comm_state = STATE_WAIT_HANDSHAKE;
            break;
    }
}

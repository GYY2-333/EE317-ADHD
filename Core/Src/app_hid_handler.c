#include "app_hid_handler.h"
#include "main.h"
#include "usbd_customhid.h"
#include <stdint.h>
#include <string.h>

#define HID_REPORT_SIZE          64U
#define SAMPLES_PER_FRAME        30U
#define ADC_PING_PONG_SIZE       (SAMPLES_PER_FRAME * 2U)
#define IMPEDANCE_CHANNEL_COUNT   6U
#define ADC_HISTORY_SIZE        128U
#define ADC_HISTORY_MASK        (ADC_HISTORY_SIZE - 1U)
#define HID_COMMAND_QUEUE_DEPTH   4U

/* ========== 通信状态 ========== */
volatile CommState comm_state = STATE_WAIT_HANDSHAKE;
volatile WorkMode current_mode = MODE_SIGNAL;

/* ========== HID 缓冲区 ========== */
uint8_t hid_in_buffer[HID_REPORT_SIZE];
static uint8_t hid_command_queue[HID_COMMAND_QUEUE_DEPTH][HID_REPORT_SIZE];
static uint16_t hid_command_length[HID_COMMAND_QUEUE_DEPTH];
static volatile uint8_t hid_command_read_idx = 0U;
static volatile uint8_t hid_command_write_idx = 0U;
static volatile uint8_t hid_command_count = 0U;

/* ========== ADC 乒乓缓冲区：前后各 30 点 ========== */
static volatile uint16_t adc_ping_pong[ADC_PING_PONG_SIZE];
static volatile uint8_t adc_write_pos = 0U;
static volatile int8_t adc_ready_half = -1;

/*
 * 阻抗模式使用一个 ADC 通道的历史数据构造六个相位延迟通道。
 * 延迟值忠实保留 PIC 参考实现；其 5Hz 推导与 10Hz 激励仍需实机标定。
 */
static uint16_t adc_history[ADC_HISTORY_SIZE] = {0};
static uint8_t history_idx = 0U;
static const uint8_t impedance_delay_samples[IMPEDANCE_CHANNEL_COUNT] = {
    0U, 13U, 25U, 38U, 50U, 63U
};

/* ========== USB 帧状态和计数器 ========== */
static volatile uint8_t frame_send_pending = 0U;
static uint8_t frame_seq = 0U;
static uint8_t type_id = 0x80U;
static uint16_t tick_count = 0U;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* 阻抗检测握手应答包：0xF8 后收到 0x80 时发送。 */
const uint8_t handshake_response[HID_REPORT_SIZE] = {
    0x77, 0x62, 0x11, 0x00, 0x57, 0xD3, 0x1F, 0x25,
    0x00, 0x00, 0x00, 0x00, 0x96, 0x35, 0xFB, 0x67,
    0x21, 0x70, 0xC4, 0x35, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0x59, 0x86, 0x69, 0x84,
    0x35, 0x8F, 0x44, 0xA9, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x0D, 0xC7, 0x0B, 0xC5, 0xB8, 0xA7, 0x9A, 0x02,
};

/* 信号检测握手应答包：默认模式收到 0x80 时发送。 */
const uint8_t signal_handshake_response[HID_REPORT_SIZE] = {
    0x77, 0x62, 0x11, 0x00, 0x4B, 0xD4, 0x9D, 0x27,
    0x00, 0x00, 0x00, 0x00, 0x86, 0xE6, 0x7E, 0xEA,
    0x51, 0xED, 0x48, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xB9, 0x85, 0x64, 0x86,
    0x84, 0x8F, 0xF1, 0xA5, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xC0, 0x19, 0xBE, 0xAC, 0xC9, 0x6C, 0x00,
};

static void ResetFrameCounters(void)
{
    frame_seq = 0U;
    type_id = 0x80U;
    tick_count = 0U;
}

static void AdvanceFrameCounters(void)
{
    uint16_t tick_step;
    uint32_t next_tick;

    frame_seq++;
    if(frame_seq == 0U)
    {
        type_id++;
        if(type_id > 0x8FU)
        {
            type_id = 0x80U;
        }
    }

    tick_step = (current_mode == MODE_IMPEDANCE) ? 30U : 10U;
    next_tick = (uint32_t)tick_count + tick_step;
    tick_count = (next_tick > 0x07FFU) ? 0U : (uint16_t)next_tick;
}

static void ResetADCBuffers(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    adc_write_pos = 0U;
    adc_ready_half = -1;
    frame_send_pending = 0U;
    history_idx = 0U;
    memset(adc_history, 0, sizeof(adc_history));

    __set_PRIMASK(primask);
}

void APP_DeviceCustomHIDReset(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    comm_state = STATE_WAIT_HANDSHAKE;
    current_mode = MODE_SIGNAL;
    hid_command_read_idx = 0U;
    hid_command_write_idx = 0U;
    hid_command_count = 0U;

    __set_PRIMASK(primask);

    ResetADCBuffers();
    ResetFrameCounters();
}

uint8_t APP_DeviceCustomHIDQueueReceivedReport(const uint8_t *report,
                                               uint16_t report_len)
{
    uint32_t primask;
    uint8_t write_idx;

    if((report == NULL) || (report_len == 0U))
    {
        return 0U;
    }
    if(report_len > HID_REPORT_SIZE)
    {
        report_len = HID_REPORT_SIZE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if(hid_command_count >= HID_COMMAND_QUEUE_DEPTH)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    write_idx = hid_command_write_idx;
    memcpy(hid_command_queue[write_idx], report, report_len);
    hid_command_length[write_idx] = report_len;

    write_idx++;
    if(write_idx >= HID_COMMAND_QUEUE_DEPTH)
    {
        write_idx = 0U;
    }
    hid_command_write_idx = write_idx;
    hid_command_count++;

    __set_PRIMASK(primask);
    return 1U;
}

uint8_t APP_DeviceCustomHIDProcessNextReport(void)
{
    uint8_t report[HID_REPORT_SIZE];
    uint16_t report_len;
    uint32_t primask;
    uint8_t read_idx;

    primask = __get_PRIMASK();
    __disable_irq();

    if(hid_command_count == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    read_idx = hid_command_read_idx;
    report_len = hid_command_length[read_idx];
    memcpy(report, hid_command_queue[read_idx], report_len);

    read_idx++;
    if(read_idx >= HID_COMMAND_QUEUE_DEPTH)
    {
        read_idx = 0U;
    }
    hid_command_read_idx = read_idx;
    hid_command_count--;

    __set_PRIMASK(primask);

    APP_DeviceCustomHIDProcessReceivedReport(report, report_len);
    return 1U;
}

/* 在 TIM2 回调中调用；阻抗模式按 PIC 参考代码生成六路相位延迟数据。 */
void APP_PushADCSample(uint16_t sample)
{
    uint8_t channel;
    uint8_t delay;
    uint8_t past;

    if(current_mode == MODE_IMPEDANCE)
    {
        adc_history[history_idx & ADC_HISTORY_MASK] = sample;

        channel = (uint8_t)(adc_write_pos % IMPEDANCE_CHANNEL_COUNT);
        delay = impedance_delay_samples[channel];
        past = (uint8_t)(history_idx - channel - delay);
        sample = adc_history[past & ADC_HISTORY_MASK];
        history_idx++;
    }

    adc_ping_pong[adc_write_pos] = sample;
    adc_write_pos++;

    /* 主循环来不及发送时，ready 始终指向最新完成的半区。 */
    if(adc_write_pos == SAMPLES_PER_FRAME)
    {
        adc_ready_half = 0;
    }
    else if(adc_write_pos >= ADC_PING_PONG_SIZE)
    {
        adc_ready_half = 1;
        adc_write_pos = 0U;
    }
}

static uint8_t CopyReadyADCFrame(void)
{
    uint32_t primask;
    int8_t ready_half;

    primask = __get_PRIMASK();
    __disable_irq();

    ready_half = adc_ready_half;
    if(ready_half < 0)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    memcpy(&hid_in_buffer[4],
           (const void *)&adc_ping_pong[(uint8_t)ready_half * SAMPLES_PER_FRAME],
           SAMPLES_PER_FRAME * sizeof(uint16_t));
    adc_ready_half = -1;

    __set_PRIMASK(primask);
    return 1U;
}

uint8_t USB_BuildAndSendDataFrame(void)
{
    if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0U;
    }

    if(frame_send_pending == 0U)
    {
        if(CopyReadyADCFrame() == 0U)
        {
            return 0U;
        }

        hid_in_buffer[0] = frame_seq;
        hid_in_buffer[1] = type_id;
        hid_in_buffer[2] = (uint8_t)(tick_count & 0xFFU);
        hid_in_buffer[3] = (uint8_t)(tick_count >> 8);
        frame_send_pending = 1U;
    }

    if(USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,
                                  hid_in_buffer,
                                  HID_REPORT_SIZE) == USBD_OK)
    {
        frame_send_pending = 0U;
        AdvanceFrameCounters();
        return 1U;
    }

    return 0U;
}

static uint8_t USB_SendHandshakeResponse(void)
{
    const uint8_t *response;

    response = (current_mode == MODE_IMPEDANCE)
             ? handshake_response
             : signal_handshake_response;
    memcpy(hid_in_buffer, response, HID_REPORT_SIZE);
    return (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,
                                       hid_in_buffer,
                                       HID_REPORT_SIZE) == USBD_OK) ? 1U : 0U;
}

void APP_DeviceCustomHIDProcessReceivedReport(const uint8_t *report,
                                              uint16_t report_len)
{
    uint8_t cmd;

    if((report == NULL) || (report_len < 1U))
    {
        return;
    }

    cmd = report[0];

    /*
     * PIC 协议中 0xF8 在所有状态下切换工作模式，且不发送应答。
     * STM32 在热切换时先退出流状态，避免混合两种 SDADC 配置的数据。
     */
    if(cmd == 0xF8U)
    {
        current_mode = (current_mode == MODE_IMPEDANCE)
                     ? MODE_SIGNAL
                     : MODE_IMPEDANCE;

        if(comm_state != STATE_WAIT_HANDSHAKE)
        {
            comm_state = STATE_WAIT_HANDSHAKE;
            ResetADCBuffers();
            ResetFrameCounters();
        }
        return;
    }

    switch(comm_state)
    {
        case STATE_WAIT_HANDSHAKE:
            if(cmd == 0x80U)
            {
                if(USB_SendHandshakeResponse() != 0U)
                {
                    comm_state = STATE_SEND_RESPONSE;
                }
            }
            break;

        case STATE_SEND_RESPONSE:
            if(cmd == 0x80U)
            {
                (void)USB_SendHandshakeResponse();
            }
            else if(cmd == 0x8FU)
            {
                ResetADCBuffers();
                ResetFrameCounters();
                comm_state = STATE_START_STREAMING;
            }
            break;

        case STATE_START_STREAMING:
        case STATE_STREAMING:
            if(cmd == 0x80U)
            {
                comm_state = STATE_WAIT_HANDSHAKE;
                ResetADCBuffers();
                if(USB_SendHandshakeResponse() != 0U)
                {
                    comm_state = STATE_SEND_RESPONSE;
                }
            }
            break;

        default:
            comm_state = STATE_WAIT_HANDSHAKE;
            break;
    }
}

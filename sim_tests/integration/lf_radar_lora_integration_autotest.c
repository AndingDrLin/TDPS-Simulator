#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lf_app.h"
#include "lf_config.h"
#include "lf_platform.h"
#include "wireless_hooks.h"
#include "wl_config.h"
#include "wl_lora.h"
#include "wl_stub_test.h"

#define LF_RADAR_RX_BUF_SIZE 512U

static uint32_t s_lf_ms = 0U;
static int16_t s_last_left_cmd = 0;
static int16_t s_last_right_cmd = 0;
static bool s_start_pressed_once = true;

static uint8_t s_radar_rx_buf[LF_RADAR_RX_BUF_SIZE];
static uint16_t s_radar_rx_len = 0U;
static uint16_t s_radar_rx_pos = 0U;

static int s_failures = 0;

static void check_true(bool cond, const char *msg)
{
    if (!cond) {
        s_failures += 1;
        printf("[FAIL] %s\n", msg);
    }
}

static void radar_append_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t copy_len;

    if (data == NULL || len == 0U || s_radar_rx_len >= LF_RADAR_RX_BUF_SIZE) {
        return;
    }

    copy_len = len;
    if ((uint16_t)(s_radar_rx_len + copy_len) > LF_RADAR_RX_BUF_SIZE) {
        copy_len = (uint16_t)(LF_RADAR_RX_BUF_SIZE - s_radar_rx_len);
    }

    memcpy(&s_radar_rx_buf[s_radar_rx_len], data, copy_len);
    s_radar_rx_len = (uint16_t)(s_radar_rx_len + copy_len);
}

static void radar_inject_frame(uint16_t distance_mm, bool has_target)
{
    uint8_t frame[8];

    frame[0] = 0xF4U;
    frame[1] = 0xF3U;
    frame[2] = 0xF2U;
    frame[3] = 0xF1U;
    frame[4] = (uint8_t)(distance_mm & 0xFFU);
    frame[5] = (uint8_t)((distance_mm >> 8) & 0xFFU);
    frame[6] = has_target ? 1U : 0U;
    frame[7] = (uint8_t)(frame[4] ^ frame[5] ^ frame[6]);

    radar_append_bytes(frame, (uint16_t)sizeof(frame));
}

static void run_steps(uint32_t ms)
{
    uint32_t i;
    for (i = 0U; i < ms; ++i) {
        LF_App_RunStep();
        Wireless_Hooks_Tick();
        LF_Platform_DelayMs(1U);
        WL_Stub_AdvanceMillis(1U);
    }
}

void LF_Platform_BoardInit(void)
{
    s_lf_ms = 0U;
    s_last_left_cmd = 0;
    s_last_right_cmd = 0;
    s_start_pressed_once = true;
    s_radar_rx_len = 0U;
    s_radar_rx_pos = 0U;
}

uint32_t LF_Platform_GetMillis(void)
{
    return s_lf_ms;
}

void LF_Platform_DelayMs(uint32_t ms)
{
    s_lf_ms += ms;
}

void LF_Platform_ReadLineSensorRaw(uint16_t out_raw[LF_SENSOR_COUNT])
{
    uint32_t i;

    if (out_raw == NULL) {
        return;
    }

    if (s_lf_ms < 3300U) {
        uint16_t base = (((s_lf_ms / 120U) & 0x1U) == 0U) ? 850U : 3300U;
        for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
            out_raw[i] = (uint16_t)(base + i * 12U);
        }
        return;
    }

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        out_raw[i] = 700U;
    }

    out_raw[(LF_SENSOR_COUNT / 2U) - 1U] = 3200U;
    out_raw[LF_SENSOR_COUNT / 2U] = 3200U;
    if (LF_SENSOR_COUNT >= 8U) {
        out_raw[(LF_SENSOR_COUNT / 2U) - 2U] = 2200U;
        out_raw[(LF_SENSOR_COUNT / 2U) + 1U] = 2200U;
    }
}

uint16_t LF_Platform_RadarRead(uint8_t *out_buf, uint16_t max_len)
{
    uint16_t remaining;
    uint16_t count;

    if (out_buf == NULL || max_len == 0U || s_radar_rx_pos >= s_radar_rx_len) {
        return 0U;
    }

    remaining = (uint16_t)(s_radar_rx_len - s_radar_rx_pos);
    count = (remaining < max_len) ? remaining : max_len;
    memcpy(out_buf, &s_radar_rx_buf[s_radar_rx_pos], count);
    s_radar_rx_pos = (uint16_t)(s_radar_rx_pos + count);

    if (s_radar_rx_pos >= s_radar_rx_len) {
        s_radar_rx_len = 0U;
        s_radar_rx_pos = 0U;
    }

    return count;
}

void LF_Platform_SetMotorCommand(int16_t left_cmd, int16_t right_cmd)
{
    s_last_left_cmd = left_cmd;
    s_last_right_cmd = right_cmd;
}

void LF_Platform_SetStatusLed(bool on)
{
    (void)on;
}

bool LF_Platform_IsStartButtonPressed(void)
{
    if (s_start_pressed_once) {
        s_start_pressed_once = false;
        return true;
    }
    return false;
}

void LF_Platform_DebugPrint(const char *msg)
{
    (void)msg;
}

int main(void)
{
    uint32_t baseline_tx_count;
    uint32_t i;
    const LF_AppContext *ctx;
    const WL_LoRa_LinkStatus *link;

    printf("[lf_radar_lora_integration_autotest] start\n");

    LF_Platform_BoardInit();
    check_true(Wireless_Hooks_Init(), "wireless hooks init should succeed in stub mode");
    LF_App_Init();

    run_steps(3600U);

    ctx = LF_App_GetContext();
    check_true(ctx->state == LF_APP_STATE_RUNNING, "app should enter RUNNING state");

    baseline_tx_count = WL_Stub_GetTxCount();
    LF_App_NotifyCheckpoint(g_wl_config.checkpoint_arch_2_1);
    run_steps(80U);

    check_true(WL_Stub_GetTxCount() > baseline_tx_count,
               "checkpoint should trigger LoRa TX path");
    {
        static const char expected_payload[] = "TEAM=15,NAME=TDPS,CP=21,TIME=00:00\n";
        uint8_t last_tx[WL_UART_TX_BUF_SIZE];
        uint16_t last_tx_len = WL_Stub_GetLastTx(last_tx, (uint16_t)sizeof(last_tx));
        check_true(last_tx_len == (uint16_t)strlen(expected_payload) &&
                   memcmp(last_tx, expected_payload, strlen(expected_payload)) == 0,
                   "checkpoint payload should use task-required MM:SS time format");
    }

    for (i = 0U; i < g_lf_config.radar_debounce_frames; ++i) {
        radar_inject_frame(320U, true);
        run_steps(20U);
    }

    ctx = LF_App_GetContext();
    check_true(ctx->obstacle_state == LF_RADAR_OBSTACLE_BLOCK,
               "radar danger frames should trigger BLOCK");
    check_true(s_last_left_cmd == 0 && s_last_right_cmd == 0,
               "BLOCK state should stop chassis command");

    for (i = 0U; i < g_lf_config.radar_debounce_frames; ++i) {
        radar_inject_frame(900U, false);
        run_steps(20U);
    }

    ctx = LF_App_GetContext();
    check_true(ctx->obstacle_state == LF_RADAR_OBSTACLE_CLEAR,
               "release frames should clear BLOCK state");

    link = WL_LoRa_GetLinkStatus();
    check_true(link->tx_success_count >= 1U,
               "integration run should have at least one successful LoRa tx");

    if (s_failures != 0) {
        printf("[lf_radar_lora_integration_autotest] failed: %d case(s)\n", s_failures);
        return 1;
    }

    printf("[lf_radar_lora_integration_autotest] all passed\n");
    return 0;
}

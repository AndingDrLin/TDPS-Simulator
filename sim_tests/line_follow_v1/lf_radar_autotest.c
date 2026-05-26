#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lf_config.h"
#include "lf_radar.h"

#define RX_BUF_SIZE 512U

static uint8_t s_rx_buf[RX_BUF_SIZE];
static uint16_t s_rx_len = 0U;
static uint16_t s_rx_pos = 0U;
static int s_failures = 0;

static void check_true(bool cond, const char *msg)
{
    if (!cond) {
        s_failures += 1;
        printf("[FAIL] %s\n", msg);
    }
}

static void append_rx(const uint8_t *data, uint16_t len)
{
    uint16_t copy_len;

    if (data == NULL || len == 0U) {
        return;
    }

    if (s_rx_len >= RX_BUF_SIZE) {
        return;
    }

    copy_len = len;
    if ((uint16_t)(s_rx_len + copy_len) > RX_BUF_SIZE) {
        copy_len = (uint16_t)(RX_BUF_SIZE - s_rx_len);
    }

    if (copy_len > 0U) {
        memcpy(&s_rx_buf[s_rx_len], data, copy_len);
        s_rx_len = (uint16_t)(s_rx_len + copy_len);
    }
}

static void inject_frame(uint16_t distance_mm, bool has_target, bool valid_checksum)
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

    if (!valid_checksum) {
        frame[7] ^= 0x5AU;
    }

    append_rx(frame, (uint16_t)sizeof(frame));
}

uint16_t LF_Platform_RadarRead(uint8_t *out_buf, uint16_t max_len)
{
    uint16_t remaining;
    uint16_t count;

    if (out_buf == NULL || max_len == 0U || s_rx_pos >= s_rx_len) {
        return 0U;
    }

    remaining = (uint16_t)(s_rx_len - s_rx_pos);
    count = (remaining < max_len) ? remaining : max_len;
    memcpy(out_buf, &s_rx_buf[s_rx_pos], count);
    s_rx_pos = (uint16_t)(s_rx_pos + count);

    if (s_rx_pos >= s_rx_len) {
        s_rx_len = 0U;
        s_rx_pos = 0U;
    }

    return count;
}

int main(void)
{
    uint32_t now = 0U;
    uint32_t i;
    const LF_RadarState *st;

    printf("[lf_radar_autotest] start\n");

    LF_Radar_Init();

    inject_frame(380U, true, false);
    LF_Radar_Tick(now += 10U);
    st = LF_Radar_GetState();
    check_true(st->parse_error_count == 1U, "invalid checksum should increase parse_error_count");

    for (i = 0U; i < g_lf_config.radar_debounce_frames; ++i) {
        inject_frame(320U, true, true);
        LF_Radar_Tick(now += 10U);
    }
    st = LF_Radar_GetState();
    check_true(st->obstacle_state == LF_RADAR_OBSTACLE_BLOCK, "danger frames should trigger BLOCK");

    LF_Radar_Tick(now += g_lf_config.radar_frame_timeout_ms + 1U);
    st = LF_Radar_GetState();
    check_true(st->obstacle_state == LF_RADAR_OBSTACLE_CLEAR, "frame timeout should clear obstacle state");

    for (i = 0U; i < g_lf_config.radar_debounce_frames; ++i) {
        inject_frame(300U, true, true);
        LF_Radar_Tick(now += 10U);
    }
    for (i = 0U; i < g_lf_config.radar_debounce_frames; ++i) {
        inject_frame(900U, false, true);
        LF_Radar_Tick(now += 10U);
    }
    st = LF_Radar_GetState();
    check_true(st->obstacle_state == LF_RADAR_OBSTACLE_CLEAR,
               "release frames should clear BLOCK state");

    if (s_failures != 0) {
        printf("[lf_radar_autotest] failed: %d case(s)\n", s_failures);
        return 1;
    }

    printf("[lf_radar_autotest] all passed\n");
    return 0;
}

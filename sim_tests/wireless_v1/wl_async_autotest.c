#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "wl_config.h"
#include "wl_lora.h"
#include "wl_platform.h"
#include "wl_stub_test.h"

static int s_failures = 0;

static void check_true(bool cond, const char *msg)
{
    if (!cond) {
        s_failures += 1;
        printf("[FAIL] %s\n", msg);
    }
}

static void reset_fixture(bool ack_enable)
{
    WL_Platform_Init();
    WL_LoRa_ServiceInit();
    WL_LoRa_SetAckEnabled(ack_enable);
    WL_Stub_SetAuxReady(true);
    WL_Stub_ClearLastTx();
    WL_Stub_ResetTxCount();
}

static void test_enqueue_and_send_without_ack(void)
{
    const WL_LoRa_LinkStatus *link;
    WL_LoRa_Status st;

    reset_fixture(false);

    st = WL_LoRa_EnqueueString("PING");
    check_true(st == WL_LORA_OK, "enqueue should succeed");

    WL_LoRa_Tick();

    link = WL_LoRa_GetLinkStatus();
    check_true(link->tx_success_count == 1U, "tx_success_count should be 1 without ACK");
    check_true(link->tx_fail_count == 0U, "tx_fail_count should be 0 without ACK");
    check_true(link->queue_depth == 0U, "queue_depth should return to 0");
    check_true(WL_Stub_GetTxCount() == 1U, "one UART transmission expected");
}

static void test_ack_timeout_and_retry(void)
{
    uint32_t i;
    const WL_LoRa_LinkStatus *link;

    reset_fixture(true);

    check_true(WL_LoRa_EnqueueString("RETRY") == WL_LORA_OK, "enqueue retry payload");

    /* 首次发送，进入等待 ACK。 */
    WL_LoRa_Tick();

    for (i = 0U; i < 16U; ++i) {
        WL_Stub_AdvanceMillis(g_wl_config.ack_timeout_ms + 1U);
        WL_LoRa_Tick();

        link = WL_LoRa_GetLinkStatus();
        if (link->tx_fail_count > 0U &&
            link->queue_depth == 0U &&
            link->waiting_ack == false) {
            break;
        }
    }

    link = WL_LoRa_GetLinkStatus();
    check_true(link->tx_success_count == 0U, "ACK timeout path should have no success");
    check_true(link->tx_fail_count == 1U, "ACK timeout path should end with one failed tx");
    check_true(link->retry_count == g_wl_config.tx_retry_max, "retry count should reach configured max");
    check_true(WL_Stub_GetTxCount() == (uint32_t)g_wl_config.tx_retry_max + 1U,
               "UART tx count should equal initial send + retries");
}

static void test_ack_success_path(void)
{
    const WL_LoRa_LinkStatus *link;
    static const uint8_t ack_line[] = {'A', 'C', 'K', '\n'};

    reset_fixture(true);

    check_true(WL_LoRa_EnqueueString("ACKED") == WL_LORA_OK, "enqueue acked payload");

    /* 首次发送。 */
    WL_LoRa_Tick();

    /* 注入 ACK 并再次 tick。 */
    WL_Stub_InjectRxData(ack_line, (uint16_t)sizeof(ack_line));
    WL_LoRa_Tick();

    link = WL_LoRa_GetLinkStatus();
    check_true(link->tx_success_count == 1U, "ACK path should succeed");
    check_true(link->tx_fail_count == 0U, "ACK path should have no failure");
    check_true(link->waiting_ack == false, "ACK should clear waiting state");
}

static void test_aux_busy_timeout(void)
{
    uint32_t i;
    const WL_LoRa_LinkStatus *link;

    reset_fixture(false);
    WL_Stub_SetAuxReady(false);

    check_true(WL_LoRa_EnqueueString("BUSY") == WL_LORA_OK, "enqueue busy payload");

    for (i = 0U; i <= g_wl_config.tx_retry_max; ++i) {
        WL_LoRa_Tick();
        WL_Stub_AdvanceMillis(g_wl_config.tx_timeout_ms + 1U);
        WL_LoRa_Tick();
    }

    link = WL_LoRa_GetLinkStatus();
    check_true(link->tx_fail_count == 1U, "AUX busy should eventually fail");
    check_true(link->retry_count == g_wl_config.tx_retry_max, "AUX busy should consume retries");
    check_true(WL_Stub_GetTxCount() == 0U, "AUX busy should not send UART payload");
}

int main(void)
{
    printf("[wl_async_autotest] start\n");

    test_enqueue_and_send_without_ack();
    test_ack_timeout_and_retry();
    test_ack_success_path();
    test_aux_busy_timeout();

    if (s_failures != 0) {
        printf("[wl_async_autotest] failed: %d case(s)\n", s_failures);
        return 1;
    }

    printf("[wl_async_autotest] all passed\n");
    return 0;
}

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "button.h"
#include "codec.h"
#include "config.h"
#include "led.h"
#include "lib/battery/battery.h"
#include "mic.h"
#include "transport.h"
#include "usb.h"
#include "wdog_facade.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

bool is_connected = false;
bool is_charging = false;
extern bool is_off;
extern bool usb_charge;
static bool charger_state_initialized = false;
static bool charger_enabled = false;
static int64_t last_codec_error_log_ms = 0;
static int64_t last_battery_diag_log_ms = 0;

static void codec_handler(uint8_t *data, size_t len)
{
    if (!is_connected) {
        return;
    }

    int err = broadcast_audio_packets(data, len);
    if (err) {
        const int64_t now_ms = k_uptime_get();
        if ((now_ms - last_codec_error_log_ms) >= 1000) {
            last_codec_error_log_ms = now_ms;
            LOG_WRN("Audio packet broadcast backpressure (err=%d)", err);
        }
    }
}

static void mic_handler(int16_t *buffer)
{
    if (!is_connected) {
        return;
    }

    int err = codec_receive_pcm(buffer, MIC_BUFFER_SAMPLES);
    if (err) {
        const int64_t now_ms = k_uptime_get();
        if ((now_ms - last_codec_error_log_ms) >= 1000) {
            last_codec_error_log_ms = now_ms;
            LOG_WRN("PCM ingest backpressure (err=%d)", err);
        }
    }
}

void set_led_state(void)
{
    if (usb_charge) {
        is_charging = !is_charging;
        set_led_green(is_charging);
    } else {
        set_led_green(false);
    }

    if (is_off) {
        set_led_red(false);
        set_led_blue(false);
        return;
    }

    if (is_connected) {
        set_led_blue(true);
        set_led_red(false);
    } else {
        set_led_blue(false);
        set_led_red(true);
    }
}

static void apply_charger_policy(void)
{
#ifdef CONFIG_OMI_ENABLE_BATTERY
    const bool should_charge = usb_charge;
    if (charger_state_initialized && charger_enabled == should_charge) {
        return;
    }

    const int err = should_charge ? battery_charge_start() : battery_charge_stop();
    if (err) {
        LOG_WRN("Battery charge policy update failed (usb=%d, err=%d)", should_charge, err);
        return;
    }

    charger_state_initialized = true;
    charger_enabled = should_charge;
    LOG_INF("Battery charger %s (usb=%d)", should_charge ? "enabled" : "disabled", usb_charge);
#endif
}

static void log_battery_diagnostics(void)
{
#ifdef CONFIG_OMI_ENABLE_BATTERY
    const int64_t now_ms = k_uptime_get();
    if ((now_ms - last_battery_diag_log_ms) < 10000) {
        return;
    }
    last_battery_diag_log_ms = now_ms;

    uint16_t battery_millivolt = 0;
    uint8_t battery_percent = 0xFF;
    bool charging = false;
    const int mv_err = battery_get_millivolt(&battery_millivolt);
    const int pct_err = (mv_err == 0)
        ? battery_get_percentage(&battery_percent, battery_millivolt)
        : -ENOTSUP;
    const int charge_err = battery_is_charge_enabled(&charging);

    LOG_INF("Battery main diag: mv=%u pct=%u usb=%d charging=%d errs[mv=%d pct=%d chg=%d]",
            battery_millivolt,
            battery_percent,
            usb_charge ? 1 : 0,
            charging ? 1 : 0,
            mv_err,
            pct_err,
            charge_err);
#endif
}

int main(void)
{
    int err;

    LOG_INF("Booting minimal XIAO firmware");
    LOG_INF("Model: %s", CONFIG_BT_DIS_MODEL);
    LOG_INF("FW: %s", CONFIG_BT_DIS_FW_REV_STR);

    err = led_start();
    if (err) {
        LOG_ERR("LED init failed: %d", err);
        return err;
    }

    err = watchdog_init();
    if (err) {
        LOG_WRN("Watchdog init failed: %d", err);
    }

#ifdef CONFIG_OMI_ENABLE_BATTERY
    err = battery_init();
    if (err) {
        LOG_ERR("Battery init failed: %d", err);
        return err;
    }
#endif

#ifdef CONFIG_OMI_ENABLE_BUTTON
    err = button_init();
    if (err) {
        LOG_ERR("Button init failed: %d", err);
        return err;
    }
    activate_button_work();
#endif

#ifdef CONFIG_OMI_ENABLE_USB
    err = init_usb();
    if (err) {
        LOG_ERR("USB init failed: %d", err);
        return err;
    }
#endif

#ifdef CONFIG_OMI_ENABLE_BATTERY
    apply_charger_policy();
#endif

    err = transport_start();
    if (err) {
        LOG_ERR("Transport start failed: %d", err);
        return err;
    }

    set_codec_callback(codec_handler);
    err = codec_start();
    if (err) {
        LOG_ERR("Codec start failed: %d", err);
        return err;
    }

    set_mic_callback(mic_handler);
    err = mic_start();
    if (err) {
        LOG_ERR("Mic start failed: %d", err);
        return err;
    }

    LOG_INF("Minimal firmware initialized");

    while (1) {
        watchdog_feed();
        apply_charger_policy();
        log_battery_diagnostics();
        set_led_state();
        k_msleep(500);
    }

    return 0;
}

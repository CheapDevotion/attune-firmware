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

static void codec_handler(uint8_t *data, size_t len)
{
    int err = broadcast_audio_packets(data, len);
    if (err) {
        LOG_ERR("Failed to broadcast audio packets: %d", err);
    }
}

static void mic_handler(int16_t *buffer)
{
    int err = codec_receive_pcm(buffer, MIC_BUFFER_SAMPLES);
    if (err) {
        LOG_ERR("Failed to process PCM data: %d", err);
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
    err = battery_charge_start();
    if (err) {
        LOG_ERR("Battery charge init failed: %d", err);
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
        set_led_state();
        k_msleep(500);
    }

    return 0;
}

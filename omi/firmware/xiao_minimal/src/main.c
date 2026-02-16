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

static int32_t mic_agc_gain_q10 = 1024;
static uint32_t mic_agc_frame_count = 0;

static int16_t clamp_pcm16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t) value;
}

static void apply_mic_agc(int16_t *buffer, size_t sample_count)
{
#if MIC_AGC_ENABLED
    if (buffer == NULL || sample_count == 0) {
        return;
    }

    int64_t sum_abs = 0;
    int32_t peak_abs = 0;
    for (size_t i = 0; i < sample_count; i++) {
        int32_t sample = buffer[i];
        if (sample < 0) {
            sample = -sample;
        }
        sum_abs += sample;
        if (sample > peak_abs) {
            peak_abs = sample;
        }
    }

    int32_t avg_abs = (int32_t)(sum_abs / (int64_t) sample_count);
    int32_t desired_gain_q10 = 1024;
    if (avg_abs >= MIC_AGC_MIN_AVG_ABS) {
        desired_gain_q10 = (MIC_AGC_TARGET_AVG_ABS * 1024) / avg_abs;
        if (desired_gain_q10 < 1024) {
            desired_gain_q10 = 1024;
        }
        if (desired_gain_q10 > MIC_AGC_MAX_GAIN_Q10) {
            desired_gain_q10 = MIC_AGC_MAX_GAIN_Q10;
        }
    }

    // Peak-aware limiter: leave headroom to avoid clipping after AGC.
    if (peak_abs > 0) {
        int32_t max_gain_from_peak = (30000 * 1024) / peak_abs;
        if (max_gain_from_peak < 1024) {
            max_gain_from_peak = 1024;
        }
        if (desired_gain_q10 > max_gain_from_peak) {
            desired_gain_q10 = max_gain_from_peak;
        }
    }

    int32_t delta = desired_gain_q10 - mic_agc_gain_q10;
    int32_t step_percent = delta > 0 ? MIC_AGC_ATTACK_PERCENT : MIC_AGC_RELEASE_PERCENT;
    int32_t step = (delta * step_percent) / 100;
    if (step == 0 && delta != 0) {
        step = delta > 0 ? 1 : -1;
    }
    mic_agc_gain_q10 += step;

    for (size_t i = 0; i < sample_count; i++) {
        int32_t scaled = ((int32_t) buffer[i] * mic_agc_gain_q10) >> 10;
        buffer[i] = clamp_pcm16(scaled);
    }

    mic_agc_frame_count++;
    if (mic_agc_frame_count % 50 == 0) {
        LOG_INF("MIC AGC avg_abs=%d peak=%d gain_q10=%d", avg_abs, peak_abs, mic_agc_gain_q10);
    }
#else
    ARG_UNUSED(buffer);
    ARG_UNUSED(sample_count);
#endif
}

static void codec_handler(uint8_t *data, size_t len)
{
    int err = broadcast_audio_packets(data, len);
    if (err) {
        LOG_ERR("Failed to broadcast audio packets: %d", err);
    }
}

static void mic_handler(int16_t *buffer)
{
    apply_mic_agc(buffer, MIC_BUFFER_SAMPLES);
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

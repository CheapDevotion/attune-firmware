#include "mic.h"

#include <haly/nrfy_gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "config.h"
#include "led.h"
#include "nrfx_clock.h"
#include "nrfx_pdm.h"
#include "utils.h"

LOG_MODULE_REGISTER(mic, CONFIG_LOG_DEFAULT_LEVEL);

//
// Port of this code: https://github.com/Seeed-Studio/Seeed_Arduino_Mic/blob/master/src/hardware/nrf52840_adc.cpp
//

static int16_t _buffer_0[MIC_BUFFER_SAMPLES];
static int16_t _buffer_1[MIC_BUFFER_SAMPLES];
static volatile uint8_t _next_buffer_index = 0;
static volatile mix_handler _callback = NULL;
static uint8_t _mic_gain_level = MIC_GAIN_DEFAULT_LEVEL;

static uint8_t clamp_mic_gain_level(uint8_t gain_level)
{
    return gain_level > 8 ? 8 : gain_level;
}

static uint8_t mic_gain_level_to_hw(uint8_t gain_level)
{
    // Omi gain mapping: 0..8 => mute, -20, -10, 0, +6, +10, +20, +30, +40 dB
    static const uint8_t gain_map[9] = {
        0x00, // 0
        0x14, // 1
        0x1E, // 2
        0x28, // 3
        0x2E, // 4
        0x32, // 5
        0x3C, // 6 (default)
        0x46, // 7
        0x50  // 8
    };
    return gain_map[clamp_mic_gain_level(gain_level)];
}

void mic_set_gain_level(uint8_t gain_level)
{
    _mic_gain_level = clamp_mic_gain_level(gain_level);
    const uint8_t hw_gain = mic_gain_level_to_hw(_mic_gain_level);

#ifdef NRF_PDM0_S
    nrf_pdm_gain_set(NRF_PDM0_S, hw_gain, hw_gain);
#elif defined(NRF_PDM0_NS)
    nrf_pdm_gain_set(NRF_PDM0_NS, hw_gain, hw_gain);
#else
    nrf_pdm_gain_set(NRF_PDM0, hw_gain, hw_gain);
#endif

    LOG_INF("Mic gain set level=%u hw=0x%02x", _mic_gain_level, hw_gain);
}

uint8_t mic_get_gain_level(void)
{
    return _mic_gain_level;
}

static void pdm_irq_handler(nrfx_pdm_evt_t const *event)
{
    // Ignore error (how to handle?)
    if (event->error) {
        LOG_ERR("PDM error: %d", event->error);
        return;
    }

    // Assign buffer
    if (event->buffer_requested) {
        LOG_DBG("Audio buffer requested");
        if (_next_buffer_index == 0) {
            nrfx_pdm_buffer_set(_buffer_0, MIC_BUFFER_SAMPLES);
            _next_buffer_index = 1;
        } else {
            nrfx_pdm_buffer_set(_buffer_1, MIC_BUFFER_SAMPLES);
            _next_buffer_index = 0;
        }
    }

    // Release buffer
    if (event->buffer_released) {
        LOG_DBG("Audio buffer requested");
        if (_callback) {
            _callback(event->buffer_released);
        }
    }
}

int mic_start()
{

    // Start the high frequency clock
    if (!nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    }

    // Configure PDM
    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    const uint8_t initial_hw_gain = mic_gain_level_to_hw(_mic_gain_level);
    pdm_config.gain_l = initial_hw_gain;
    pdm_config.gain_r = initial_hw_gain;
    pdm_config.interrupt_priority = MIC_IRC_PRIORITY;
    pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
    pdm_config.mode = NRF_PDM_MODE_MONO;
    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    pdm_config.ratio = NRF_PDM_RATIO_80X;
    IRQ_DIRECT_CONNECT(PDM_IRQn, 5, nrfx_pdm_irq_handler, 0); // IMPORTANT!
    if (nrfx_pdm_init(&pdm_config, pdm_irq_handler) != NRFX_SUCCESS) {
        LOG_ERR("Audio unable to initialize PDM");
        return -1;
    }

    // Power on Mic
    nrfy_gpio_cfg_output(PDM_PWR_PIN);
    nrfy_gpio_pin_set(PDM_PWR_PIN);

    // Start PDM
    if (nrfx_pdm_start() != NRFX_SUCCESS) {
        LOG_ERR("Audio unable to start PDM");
        return -1;
    }

    // Re-apply selected gain after start.
    mic_set_gain_level(_mic_gain_level);

    LOG_INF("Audio microphone started");
    return 0;
}

void set_mic_callback(mix_handler callback)
{
    _callback = callback;
}

void mic_off()
{
    nrfy_gpio_pin_clear(PDM_PWR_PIN);
}

void mic_on()
{
    nrfy_gpio_pin_set(PDM_PWR_PIN);
}

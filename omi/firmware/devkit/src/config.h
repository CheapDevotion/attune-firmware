#include <haly/nrfy_gpio.h>

// #define SAMPLE_RATE 16000
#define MIC_IRC_PRIORITY 7
#define MIC_BUFFER_SAMPLES 1600    // 100ms
#define AUDIO_BUFFER_SAMPLES 16000 // 1s
#define NETWORK_RING_BUF_SIZE 32   // number of frames * CODEC_OUTPUT_MAX_BYTES
#define MINIMAL_PACKET_SIZE 100    // Less than that doesn't make sence to send anything at all

// Omi-style discrete mic gain levels (0..8).
#define MIC_GAIN_DEFAULT_LEVEL 6

// Omi-style capture path: fixed mic gain level, no firmware AGC.
#define MIC_AGC_ENABLED 0
#define MIC_AGC_TARGET_AVG_ABS 2200
#define MIC_AGC_MIN_AVG_ABS 180
#define MIC_AGC_MAX_GAIN_Q10 (3 * 1024)
#define MIC_AGC_ATTACK_PERCENT 12
#define MIC_AGC_RELEASE_PERCENT 35

// PIN definitions
// https://github.com/Seeed-Studio/Adafruit_nRF52_Arduino/blob/5aa3573913449410fd60f76b75673c53855ff2ec/variants/Seeed_XIAO_nRF52840_Sense/variant.cpp#L34
#define PDM_DIN_PIN NRF_GPIO_PIN_MAP(0, 16)
#define PDM_CLK_PIN NRF_GPIO_PIN_MAP(1, 0)
#define PDM_PWR_PIN NRF_GPIO_PIN_MAP(1, 10)

// Codecs
#ifdef CONFIG_OMI_CODEC_OPUS
#define CODEC_OPUS 1
#else
#error "Enable CONFIG_OMI_CODEC_OPUS in the project .conf file"
#endif

#if CODEC_OPUS
#define CODEC_PACKAGE_SAMPLES 160
#define CODEC_OUTPUT_MAX_BYTES CODEC_PACKAGE_SAMPLES * 2 // Let's assume that 16bit is enough
#define CODEC_OPUS_APPLICATION OPUS_APPLICATION_RESTRICTED_LOWDELAY
#define CODEC_OPUS_BITRATE 40000
#define CODEC_OPUS_VBR 1 // Or 1
#define CODEC_OPUS_COMPLEXITY 5
#endif
#define CONFIG_OPUS_MODE CONFIG_OPUS_MODE_CELT

// Codec IDs

#ifdef CODEC_OPUS
#define CODEC_ID 20
#endif

// Logs
// #define LOG_DISCARDED

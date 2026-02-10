#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include "speaker.h"

int speaker_init(void) { return 0; }
uint16_t speak(uint16_t len, const void *buf)
{
    ARG_UNUSED(buf);
    return len;
}
int play_boot_sound(void) { return 0; }
int init_haptic_pin(void) { return 0; }
void play_haptic_milli(uint32_t duration) { ARG_UNUSED(duration); }
void speaker_off(void) {}
void register_speaker_service(void) {}

int mount_sd_card(void) { return 0; }
void sd_on(void) {}
void sd_off(void) {}
bool is_sd_on(void) { return false; }
int write_to_file(uint8_t *buf, uint32_t len)
{
    ARG_UNUSED(buf);
    ARG_UNUSED(len);
    return 0;
}
uint32_t get_file_size(uint8_t file_num)
{
    ARG_UNUSED(file_num);
    return 0;
}
int get_offset(void) { return 0; }

int storage_init(void) { return 0; }

bool storage_is_on = false;
uint8_t file_count = 0;
uint32_t file_num_array[2] = {0};

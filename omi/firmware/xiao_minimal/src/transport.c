#include "transport.h"

#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include "button.h"
#include "codec.h"
#include "config.h"
#include "lib/battery/battery.h"
#include "mic.h"
#include "usb.h"
#include "utils.h"

LOG_MODULE_REGISTER(transport, CONFIG_LOG_DEFAULT_LEVEL);

extern bool is_connected;
extern bool is_off;
extern bool usb_charge;

struct bt_conn *current_connection = NULL;
uint16_t current_mtu = 0;

static void audio_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

static ssize_t audio_data_read_characteristic(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf,
                                              uint16_t len,
                                              uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t audio_codec_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset)
{
    uint8_t value[1] = {CODEC_ID};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static ssize_t audio_mic_gain_read_characteristic(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attr,
                                                  void *buf,
                                                  uint16_t len,
                                                  uint16_t offset)
{
    ARG_UNUSED(attr);
    const uint8_t level = mic_get_gain_level();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &level, sizeof(level));
}

static ssize_t audio_mic_gain_write_characteristic(struct bt_conn *conn,
                                                   const struct bt_gatt_attr *attr,
                                                   const void *buf,
                                                   uint16_t len,
                                                   uint16_t offset,
                                                   uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (buf == NULL || len < 1) {
        return 0;
    }

    const uint8_t requested = ((const uint8_t *)buf)[0];
    mic_set_gain_level(requested);
    return len;
}

enum {
    AUDIO_CTRL_CMD_PAUSE_STREAM = 0x01,
    AUDIO_CTRL_CMD_RESUME_STREAM = 0x02,
    AUDIO_CTRL_CMD_ENTER_DEEP_SLEEP = 0x03,
    AUDIO_CTRL_CMD_NOTIFY_STATUS = 0x04,
};

enum {
    AUDIO_STATUS_FLAG_USB_POWER = 1 << 0,
    AUDIO_STATUS_FLAG_CHARGING_ENABLED = 1 << 1,
    AUDIO_STATUS_FLAG_STREAM_PAUSED = 1 << 2,
};

static volatile bool stream_paused = false;
static bool battery_filter_initialized = false;
static uint16_t battery_filtered_mv = 0;
static uint8_t battery_filtered_percent = 0xFF;
static int64_t battery_last_update_ms = 0;

static uint8_t stabilize_battery_percent(uint8_t raw_percent, uint16_t raw_mv)
{
    if (raw_percent == 0xFF) {
        return battery_filter_initialized ? battery_filtered_percent : raw_percent;
    }

    const int64_t now_ms = k_uptime_get();
    if (!battery_filter_initialized) {
        battery_filter_initialized = true;
        battery_filtered_mv = raw_mv;
        battery_filtered_percent = raw_percent;
        battery_last_update_ms = now_ms;
        return battery_filtered_percent;
    }

    if (raw_mv > 0) {
        // Exponential moving average to reduce load-induced voltage swings.
        battery_filtered_mv = (uint16_t)(((uint32_t)battery_filtered_mv * 7U + raw_mv) / 8U);
    }

    uint8_t candidate = raw_percent;
    if (battery_filtered_mv > 0) {
        (void)battery_get_percentage(&candidate, battery_filtered_mv);
    }

    int elapsed_s = (int)((now_ms - battery_last_update_ms) / 1000);
    if (elapsed_s < 1) {
        elapsed_s = 1;
    }

    const int prev = (int)battery_filtered_percent;
    const int max_rise = MIN(24, 2 + elapsed_s);
    const int max_fall = MIN(28, 3 + (elapsed_s * 2));
    int delta = (int)candidate - prev;
    if (delta > max_rise) {
        candidate = (uint8_t)(prev + max_rise);
    } else if (-delta > max_fall) {
        candidate = (uint8_t)(prev - max_fall);
    }

    battery_filtered_percent = candidate;
    battery_last_update_ms = now_ms;
    return battery_filtered_percent;
}

static ssize_t audio_control_write_characteristic(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attr,
                                                  const void *buf,
                                                  uint16_t len,
                                                  uint16_t offset,
                                                  uint8_t flags);
static ssize_t audio_status_read_characteristic(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                void *buf,
                                                uint16_t len,
                                                uint16_t offset);
static void audio_status_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static void audio_notify_status(struct bt_conn *conn);
static void audio_build_status_payload(uint8_t payload[5]);

static struct bt_uuid_128 audio_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10000, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_data_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10001, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_format_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10002, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_mic_gain_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10003, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_control_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10004, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10005, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

static struct bt_gatt_attr audio_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&audio_service_uuid),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           audio_data_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CCC(audio_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_format_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           audio_codec_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_mic_gain_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           audio_mic_gain_read_characteristic,
                           audio_mic_gain_write_characteristic,
                           NULL),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_control_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           audio_control_write_characteristic,
                           NULL),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_status_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           audio_status_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CCC(audio_status_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service audio_service = BT_GATT_SERVICE(audio_service_attr);

static struct bt_uuid_128 dfu_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00001530, 0x1212, 0xEFDE, 0x1523, 0x785FEABCD123));
static struct bt_uuid_128 dfu_control_point_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00001531, 0x1212, 0xEFDE, 0x1523, 0x785FEABCD123));

static void dfu_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

static ssize_t dfu_control_point_write_handler(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf,
                                               uint16_t len,
                                               uint16_t offset,
                                               uint8_t flags)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len == 1 && ((uint8_t *)buf)[0] == 0x06) {
        NRF_POWER->GPREGRET = 0xA8;
        NVIC_SystemReset();
    } else if (len == 2 && ((uint8_t *)buf)[0] == 0x01) {
        uint8_t notification_value = 0x10;
        bt_gatt_notify(conn, &audio_service.attrs[1], &notification_value, sizeof(notification_value));
        NRF_POWER->GPREGRET = 0xA8;
        NVIC_SystemReset();
    }
    return len;
}

static struct bt_gatt_attr dfu_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&dfu_service_uuid),
    BT_GATT_CHARACTERISTIC(&dfu_control_point_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           dfu_control_point_write_handler,
                           NULL),
    BT_GATT_CCC(dfu_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service dfu_service = BT_GATT_SERVICE(dfu_service_attr);

static const struct bt_data bt_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_UUID128_ALL, audio_service_uuid.val, sizeof(audio_service_uuid.val)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data bt_sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
    BT_DATA(BT_DATA_UUID128_ALL, dfu_service_uuid.val, sizeof(dfu_service_uuid.val)),
};

static void _transport_connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info = {0};
    if (err) {
        return;
    }

    if (bt_conn_get_info(conn, &info)) {
        return;
    }

    if (current_connection != NULL) {
        bt_conn_unref(current_connection);
    }
    current_connection = bt_conn_ref(conn);
    current_mtu = info.le.data_len->tx_max_len;
    stream_paused = false;
    mic_on();
    is_connected = true;
    audio_notify_status(conn);
}

static void _transport_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);
    is_connected = false;

    if (current_connection != NULL) {
        bt_conn_unref(current_connection);
        current_connection = NULL;
    }
    current_mtu = 0;
}

static void _le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    ARG_UNUSED(conn);
    current_mtu = info->tx_max_len;
}

static struct bt_conn_cb _callback_references = {
    .connected = _transport_connected,
    .disconnected = _transport_disconnected,
    .le_data_len_updated = _le_data_length_updated,
};

#define NET_BUFFER_HEADER_SIZE 3
#define RING_BUFFER_HEADER_SIZE 2
static uint8_t tx_queue[NETWORK_RING_BUF_SIZE * (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)];
static uint8_t tx_buffer[CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE];
static uint8_t tx_buffer_2[CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE];
static uint8_t pusher_temp_data[CODEC_OUTPUT_MAX_BYTES + NET_BUFFER_HEADER_SIZE];
static uint32_t tx_buffer_size = 0;
static struct ring_buf ring_buf;
static uint16_t packet_next_index = 0;

static bool write_to_tx_queue(uint8_t *data, size_t size)
{
    if (size > CODEC_OUTPUT_MAX_BYTES) {
        return false;
    }

    tx_buffer_2[0] = size & 0xFF;
    tx_buffer_2[1] = (size >> 8) & 0xFF;
    memcpy(tx_buffer_2 + RING_BUFFER_HEADER_SIZE, data, size);

    int written = ring_buf_put(&ring_buf, tx_buffer_2, CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE);
    return written == (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE);
}

static bool read_from_tx_queue(void)
{
    tx_buffer_size = ring_buf_get(&ring_buf, tx_buffer, CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE);
    if (tx_buffer_size != (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)) {
        return false;
    }
    tx_buffer_size = tx_buffer[0] + (tx_buffer[1] << 8);
    return true;
}

static void audio_status_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

static void audio_build_status_payload(uint8_t payload[5])
{
    uint16_t battery_millivolt = 0;
    uint8_t battery_percent = 0xFF;
    bool charge_enabled = false;
    uint8_t flags = 0;

    if (usb_charge) {
        flags |= AUDIO_STATUS_FLAG_USB_POWER;
    }
    if (stream_paused) {
        flags |= AUDIO_STATUS_FLAG_STREAM_PAUSED;
    }

#ifdef CONFIG_OMI_ENABLE_BATTERY
    if (battery_get_millivolt(&battery_millivolt) == 0) {
        (void)battery_get_percentage(&battery_percent, battery_millivolt);
        battery_percent = stabilize_battery_percent(battery_percent, battery_millivolt);
    }
    if (battery_is_charge_enabled(&charge_enabled) == 0 && charge_enabled) {
        flags |= AUDIO_STATUS_FLAG_CHARGING_ENABLED;
    }
#else
    ARG_UNUSED(charge_enabled);
#endif

    payload[0] = 1; // payload version
    payload[1] = flags;
    payload[2] = battery_percent;
    payload[3] = (uint8_t)(battery_millivolt & 0xFF);
    payload[4] = (uint8_t)((battery_millivolt >> 8) & 0xFF);
}

static void audio_notify_status(struct bt_conn *conn)
{
    uint8_t payload[5] = {0};
    audio_build_status_payload(payload);

    int err = bt_gatt_notify_uuid(conn, &audio_characteristic_status_uuid.uuid, audio_service.attrs, payload, sizeof(payload));
    if (err && err != -ENOTCONN) {
        LOG_DBG("Audio status notify failed: %d", err);
    }
}

static ssize_t audio_status_read_characteristic(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                void *buf,
                                                uint16_t len,
                                                uint16_t offset)
{
    uint8_t payload[5] = {0};
    audio_build_status_payload(payload);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, sizeof(payload));
}

static ssize_t audio_control_write_characteristic(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attr,
                                                  const void *buf,
                                                  uint16_t len,
                                                  uint16_t offset,
                                                  uint8_t flags)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (buf == NULL || len < 1) {
        return 0;
    }

    const uint8_t cmd = ((const uint8_t *)buf)[0];
    switch (cmd) {
    case AUDIO_CTRL_CMD_PAUSE_STREAM:
        stream_paused = true;
        ring_buf_reset(&ring_buf);
        mic_off();
        LOG_INF("Audio stream paused");
        audio_notify_status(conn);
        break;
    case AUDIO_CTRL_CMD_RESUME_STREAM:
        mic_on();
        stream_paused = false;
        LOG_INF("Audio stream resumed");
        audio_notify_status(conn);
        break;
    case AUDIO_CTRL_CMD_ENTER_DEEP_SLEEP:
        LOG_INF("Deep sleep command received");
        stream_paused = true;
        audio_notify_status(conn);
        is_off = true;
        bt_off();
        turnoff_all();
        break;
    case AUDIO_CTRL_CMD_NOTIFY_STATUS:
        audio_notify_status(conn);
        break;
    default:
        LOG_WRN("Unknown audio control command: 0x%02X", cmd);
        break;
    }

    return len;
}

K_THREAD_STACK_DEFINE(pusher_stack, 2048);
static struct k_thread pusher_thread;

void pusher(void)
{
    while (1) {
        struct bt_conn *conn = current_connection;
        if (conn) {
            conn = bt_conn_ref(conn);
        }

        if (conn && current_mtu >= MINIMAL_PACKET_SIZE &&
            bt_gatt_is_subscribed(conn, &audio_service.attrs[1], BT_GATT_CCC_NOTIFY)) {

            if (stream_paused) {
                k_sleep(K_MSEC(5));
            } else if (read_from_tx_queue()) {
                uint8_t *buffer = tx_buffer + RING_BUFFER_HEADER_SIZE;
                uint32_t offset = 0;
                uint8_t index = 0;

                while (offset < tx_buffer_size) {
                    uint32_t id = packet_next_index++;
                    uint32_t packet_size = MIN(current_mtu - NET_BUFFER_HEADER_SIZE, tx_buffer_size - offset);
                    pusher_temp_data[0] = id & 0xFF;
                    pusher_temp_data[1] = (id >> 8) & 0xFF;
                    pusher_temp_data[2] = index;
                    memcpy(pusher_temp_data + NET_BUFFER_HEADER_SIZE, buffer + offset, packet_size);

                    offset += packet_size;
                    index++;

                    (void)bt_gatt_notify(conn,
                                         &audio_service.attrs[1],
                                         pusher_temp_data,
                                         packet_size + NET_BUFFER_HEADER_SIZE);
                }
            }
        }

        if (conn) {
            bt_conn_unref(conn);
        }

        k_yield();
    }
}

int transport_start(void)
{
    bt_conn_cb_register(&_callback_references);

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        return err;
    }

#ifdef CONFIG_OMI_ENABLE_BUTTON
    register_button_service();
#endif

    bt_gatt_service_register(&audio_service);
    bt_gatt_service_register(&dfu_service);

    err = bt_le_adv_start(BT_LE_ADV_CONN, bt_ad, ARRAY_SIZE(bt_ad), bt_sd, ARRAY_SIZE(bt_sd));
    if (err) {
        LOG_ERR("Adv start failed: %d", err);
        return err;
    }

    ring_buf_init(&ring_buf, sizeof(tx_queue), tx_queue);
    k_thread_create(&pusher_thread,
                    pusher_stack,
                    K_THREAD_STACK_SIZEOF(pusher_stack),
                    (k_thread_entry_t)pusher,
                    NULL,
                    NULL,
                    NULL,
                    K_PRIO_PREEMPT(7),
                    0,
                    K_NO_WAIT);

    return 0;
}

int broadcast_audio_packets(uint8_t *buffer, size_t size)
{
    if (stream_paused) {
        return 0;
    }

    int retry_count = 0;
    while (retry_count < 3 && !write_to_tx_queue(buffer, size)) {
        k_sleep(K_MSEC(1));
        retry_count++;
    }
    return (retry_count >= 3) ? -1 : 0;
}

struct bt_conn *get_current_connection(void)
{
    return current_connection;
}

int bt_off(void)
{
    if (current_connection != NULL) {
        bt_conn_disconnect(current_connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(current_connection);
        current_connection = NULL;
    }

    (void)bt_le_adv_stop();
    (void)bt_disable();
    stream_paused = false;
    is_connected = false;
    current_mtu = 0;
    return 0;
}

int bt_on(void)
{
    int err = bt_enable(NULL);
    if (err) {
        return err;
    }

    stream_paused = false;
    mic_on();
    return bt_le_adv_start(BT_LE_ADV_CONN, bt_ad, ARRAY_SIZE(bt_ad), bt_sd, ARRAY_SIZE(bt_sd));
}

void accel_off(void)
{
    /* not used in minimal build */
}

#include "ble_spp_server.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *tag = "BLE_SPP";

// SPP Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
// SPP Data Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t gatt_svr_svc_spp_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

static const ble_uuid128_t gatt_svr_chr_spp_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

static uint16_t spp_handle;
static uint8_t ble_spp_connected = 0;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool spp_notify_enabled = false;

static int gatt_svr_chr_access_spp(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);

static void gatt_svr_subscribe_cb(struct ble_gap_event *event);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_spp_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &gatt_svr_chr_spp_uuid.u,
                .access_cb = gatt_svr_chr_access_spp,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &spp_handle,
            },
            {
                0, /* No more characteristics in this service */
            },
        },
    },
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_access_spp(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Handle read/write if needed
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // Can verify valid access if needed
        return 0;
    }
    // Write logic can be added here
    return 0;
}

static void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ESP_LOGI(tag, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    }
    else
    {
        ESP_LOGI(tag, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    if (event->subscribe.attr_handle == spp_handle)
    {
        spp_notify_enabled = event->subscribe.cur_notify;
        ESP_LOGI(tag, "spp notification status: %d", spp_notify_enabled);
    }
}

static void ble_spp_server_on_reset(int reason)
{
    ESP_LOGE(tag, "Resetting state; reason=%d", reason);
}

static void ble_spp_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(tag, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_spp_server_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(tag, "error enabling advertisement; rc=%d", rc);
        return;
    }
}

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(tag, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;
            ble_spp_connected = 1;
            update_ble_connection_status(true);
        }
        else
        {
            ble_spp_server_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag, "disconnect; reason=%d", event->disconnect.reason);
        ble_spp_connected = 0;
        spp_notify_enabled = false; // Reset notification status
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        update_ble_connection_status(false);
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(tag, "connection updated; status=%d",
                 event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(tag, "adv complete");
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        gatt_svr_subscribe_cb(event);
        return 0;

    default:
        return 0;
    }
}

static void ble_spp_server_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set("ESP32_SPP_SERVER");
    assert(rc == 0);

    ble_spp_server_advertise();
}

static void ble_spp_server_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_spp_server_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(tag, "Initializing NimBLE HCI. Free Internal Heap: %lu, Free SPIRAM: %lu",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(tag, "nimble_port_init failed: %d (%s)", ret, esp_err_to_name(ret));
        return;
    }

    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    // ble_hs_cfg.gap_event_cb = ble_spp_server_gap_event;
    // ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    // ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    nimble_port_freertos_init(ble_spp_server_host_task);
}

void ble_spp_server_send_data(uint8_t *data, uint16_t len)
{
    if (ble_spp_connected && spp_notify_enabled)
    {
        struct os_mbuf *om;
        om = ble_hs_mbuf_from_flat(data, len);
        if (!om)
        {
            return;
        }
        ble_gatts_notify_custom(conn_handle, spp_handle, om);
    }
}

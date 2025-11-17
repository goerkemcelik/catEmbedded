#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_log.h"

#include "ble_sens.h"

static const char *TAG = "gatt_svr";

static const char *manuf_name = "ESP32-C3";
static const char *model_num = "ADC BLE Sensor";

/* Handle für die Potentiometer-Characteristic */
uint16_t pot_value_handle;

/* Handle der aktiven Verbindung */
uint16_t pot_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* Vorwärtsdeklarationen der Access Callbacks */
static int gatt_svr_chr_access_potentiometer(uint16_t conn_handle, uint16_t attr_handle,
                                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg);

/* 
 * GATT Services Definition:
 *  - Potentiometer Service (custom / Environmental Sensing)
 *  - Device Information Service
 */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Potentiometer Measurement */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_SVR_SVC_POTENTIOMETER_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: Potentiometer Voltage (in mV) */
                .uuid = BLE_UUID16_DECLARE(GATT_SVR_CHR_POT_VALUE_UUID),
                .access_cb = gatt_svr_chr_access_potentiometer,
                .val_handle = &pot_value_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }, /* end of characteristics */
        },
    },

    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: Manufacturer Name */
                .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Characteristic: Model Number */
                .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 },
        },
    },

    { 0 }, /* end of services */
};

/* Letzter gelesener Potentiometerwert in mV */
uint16_t latest_voltage_mv = 0;

/* Access Callback: Potentiometer-Wert */
static int gatt_svr_chr_access_potentiometer(uint16_t conn_handle, uint16_t attr_handle,
                                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char buf[8];
    int len = snprintf(buf, sizeof(buf), "%u mV", latest_voltage_mv);

    int rc = os_mbuf_append(ctxt->om, buf, len);
    ESP_LOGI(TAG, "GATT read request: %u mV", latest_voltage_mv);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* Access Callback: Device Info Service */
static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_MODEL_NUMBER_UUID) {
        rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == GATT_MANUFACTURER_NAME_UUID) {
        rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

/* Registrierungscallback für Debug (wie im Template) */
void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

/* Initialisierung des GATT Servers */
int
gatt_svr_init(void)
{
    int rc;

    /* Init Standard GAP und GATT Services (aus Template) */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void blehr_sens_send(uint16_t value_mv)
{
    latest_voltage_mv = value_mv;

    if (pot_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        char buf[8];
        int len = snprintf(buf, sizeof(buf), " %u mV", value_mv);
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
        int rc = ble_gattc_notify_custom(pot_conn_handle, pot_value_handle, om);
        if (rc == 0) {
            ESP_LOGI(TAG, "Notified %u mV to BLE client", value_mv);
        } else {
            ESP_LOGW(TAG, "Notify failed: rc=%d", rc);
        }
    }
}
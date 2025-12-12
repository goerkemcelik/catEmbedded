#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "ble_sens.h"

#define TAG "MAIN"
#define POT_CHANNEL ADC1_CHANNEL_2
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define SAMPLES 100

static uint8_t own_addr_type;

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE connected");
            pot_conn_handle = event->connect.conn_handle;
        } else {
            ESP_LOGI(TAG, "BLE connect failed; retrying");
            ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                              &(struct ble_gap_adv_params){.conn_mode = BLE_GAP_CONN_MODE_UND,
                                                           .disc_mode = BLE_GAP_DISC_MODE_GEN},
                              ble_gap_event, NULL);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected; restarting advertise");
        pot_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &(struct ble_gap_adv_params){.conn_mode = BLE_GAP_CONN_MODE_UND,
                                                       .disc_mode = BLE_GAP_DISC_MODE_GEN},
                          ble_gap_event, NULL);
        return 0;

    default:
        return 0;
    }
}

static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESP32C3_CAT";
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_gap_event, NULL);

    ESP_LOGI(TAG, "Advertising started");
}

static void nimble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ESP_ERROR_CHECK(gatt_svr_init());

    nimble_port_freertos_init(nimble_host_task);

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(POT_CHANNEL, ADC_ATTEN);

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);

    while (1) {
        uint32_t sum = 0;
        for (int i = 0; i < SAMPLES; i++) {
            sum += adc1_get_raw(POT_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        int mean = sum / SAMPLES;
        uint16_t voltageMean = esp_adc_cal_raw_to_voltage(mean, &adc_chars);

        printf("{P0|Mean|%u mV}\n", voltageMean);
        ble_pot_send(voltageMean);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
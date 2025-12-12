#pragma once

#include "nimble/ble.h"
#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BLE Initialisierung */
int gatt_svr_init(void);

/* Funktion, um einen neuen Potentiometerwert per Notify zu senden */
void ble_adc_send(uint16_t value_mv);

/* Globale Connection Handles */
extern uint16_t adc_conn_handle;

/* Beispiel eigene UUIDs (Custom Service + Characteristic) */
#define GATT_SVR_SVC_ADC_UUID  0xFFF0
#define GATT_SVR_CHR_ADC_VALUE_UUID       0xFFF1

/* Device Information Service UUIDs (from official Bluetooth spec) */
#define GATT_DEVICE_INFO_UUID          0x180A
#define GATT_MANUFACTURER_NAME_UUID    0x2A29
#define GATT_MODEL_NUMBER_UUID         0x2A24


#ifdef __cplusplus
}
#endif

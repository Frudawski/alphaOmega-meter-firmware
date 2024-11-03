/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

// THE ORIGINAL EXAMPLE SOFTWARE "blehr_sens.h" FROM ESPRESSIF ESP-IDF V4.4.3 WAS CHANGED BY FREDERIC RUDAWSKI


#ifndef H_BLESERVICE_AOMETER_
#define H_BLESERVICE_AOMETER_

#include "nimble/ble.h"
#include "modlog/modlog.h"

#ifdef __cplusplus
extern "C" {
#endif

/* configuration */
#define GATT_HRS_UUID                           0x180D
#define GATT_HRS_MEASUREMENT_UUID               0x2A37
// Create ao-meter UUID128 (reverse order)
static const ble_uuid128_t GATT_AOM_UUID = BLE_UUID128_INIT(0x11, 0xfc, 0x18, 0x53, 0xae, 0x1b, 0x60, 0x98, 0xad, 0x42, 0x5d, 0x0e, 0x3f, 0x99, 0x09, 0xfc);
#define GATT_AOM_MEASUREMENT_UUID				0x2756 // radiance in W m^⁻2 sr^-1
#define GATT_HRS_BODY_SENSOR_LOC_UUID           0x2A38
#define GATT_DEVICE_INFO_UUID                   0x180A
#define GATT_MANUFACTURER_NAME_UUID             0x2A29
#define GATT_MODEL_NUMBER_UUID                  0x2A24

/* ------ BLE characteristics ? -------- */
/*
 * Device Name
 * Serial Number
 * Settings -> NVS
 * 		+ During DOSI mode? -> advertising settings data?
 * 			+ settings sync
 * 		+ masks
 * Time sync
 * 		+ read device time -> sync when deviation
 * Log data
 * 		+ only after time sync -> get rest data
 * Command control
 * 		+ send image over BLE?
 * 		+ log over BLE?
 * Calibration initialization?
 * 		+ on device calibration
 * Initialize secure BLE connection
 * 		+ PIN?
 */

extern uint16_t hrs_hrm_handle;

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);

#ifdef __cplusplus
}
#endif

#endif

/*
 * client.c
 *
 *  Created on: 14 мая 2021 г.
 *      Author: Владимир Бирюков.
 *
 *
 * Модуль обеспечивает сканирование BT устройств. При нахождении пакета от
 * сохранённого в EEPROM адреса, если сервер non-connectable, то ищется поле
 * manufacturer-specific data и оттуда забирается температура, которая
 * передаётся на отображение. Одновременно запускается таймер, по истечении
 * которого соответствующая шкала гасится. Таймер перезапускается с каждым
 * принятым адвертисментом, поэтому это событие  произойдёт если датчик стал
 * недоступен. Дальнейшее описание от предыдущей версии, но код сохранён.
 *
 * Модуль обеспечивает подключение к сенсорам. При старте запускается пассивное
 * сканирование. При получении ответа от сканирования, проверяется на адрес
 * считанный из EEPROM. Если адрес совпадает и с ним еще не создано соединение -
 * выполняется соединение. По выполнении соединения проверяется наличие сервиса
 * и характеристики по UUID и получение их хэндлов. При благополучном исходе
 * запускается таймер на 3 секунды, после отработки которого будет послан запрос
 * на считывание температуры. И если остались еще не подключенные усройства -
 * сканирование возобновляется. По получению ответа с температурой, снова
 * запускается таймер с заданным интервалом, а полученное значение отдаётся на
 * отображение.
 */

/*Simplified BSD License (FreeBSD License)
Copyright (c) 2021, Vladimir Birjukov, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "sl_bluetooth.h"
#include "app_assert.h"
#include "app_log.h"
#include "configuration.h"
#include "EEPROM.h"
#include "client.h"
#include "em_gpio.h"
#include "server.h"
#include "visualise.h"

// connection parameters
#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0

#define CONNECTION_HANDLE_INVALID     ((uint8_t)0xFFu)

typedef enum {
  scanning,
  opening,
  discover_services,
  discover_characteristics,
  not_discovered,
  enable_indication,
  discovery_finished,
  running
} conn_state_t;

typedef struct {
  uint8_t  connection_handle;
  uint32_t thermometer_service_handle;
  uint16_t thermometer_characteristic_handle;
} conn_properties_t;

static uint8_t beacon_adv_data[64];

static conn_properties_t conn_properties[2];
// State of the connection under establishment
static conn_state_t conn_state;
// Counter of active connections
static uint8_t active_connection_num = 0;
uint8_t   required_connections_num = 0;

const uint8_t service_UUID[2] =        {0x1a, 0x18},
              characteristic_UUID[2] = {0x6e, 0x2a};

static uint16_t gatt_result;

sl_sleeptimer_timer_handle_t left_channel, right_channel, connection_timeout;
uint32_t left_data = 0, right_data = 1;
uint16_t voltage[2] = {5, 5};

void not_connected_in_time(sl_sleeptimer_timer_handle_t *handle, void *timer_data);
void timer_callback(sl_sleeptimer_timer_handle_t *handle, void *timer_data);
void timer_stop(sl_sleeptimer_timer_handle_t *handle, void *timer_data);

void client_on_event(sl_bt_msg_t *evt) {
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      conn_properties[0].connection_handle = CONNECTION_HANDLE_INVALID;
      conn_properties[1].connection_handle = CONNECTION_HANDLE_INVALID;

      sc = sl_bt_scanner_set_mode(gap_1m_phy, SCAN_PASSIVE);
      app_assert_status(sc);
      // Set scan interval and scan window
      sc = sl_bt_scanner_set_timing(gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
      app_assert_status(sc);
      // Set the default connection parameters for subsequent connections
      sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN,
                                                   CONN_INTERVAL_MAX,
                                                   CONN_SLAVE_LATENCY,
                                                   CONN_TIMEOUT,
                                                   CONN_MIN_CE_LENGTH,
                                                   CONN_MAX_CE_LENGTH);
      app_assert_status(sc);
      // Start scanning - looking for thermometer devices
      if (required_connections_num) {
        sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
        app_log("Scanner started  sc=%02x\r\n", sc);
        app_assert_status_f(sc,
                            "Failed to start discovery #1\n");
        conn_state = scanning;
      }
      break;

    case sl_bt_evt_scanner_scan_report_id:
      for (unsigned int ii = 0; ii < 2; ii++) {
          if (memcmp(evt->data.evt_scanner_scan_report.address.addr, data[ii].mac_addr.addr, 6) == 0) {
              if (conn_properties[ii].connection_handle == CONNECTION_HANDLE_INVALID) {
                  if ((evt->data.evt_scanner_scan_report.packet_type & 0x07) == 0x03) { // non-connectable
                      int16_t remote_temperature=0;
                      uint8_t data_size = evt->data.evt_scanner_scan_report.data.len;
                      memcpy(beacon_adv_data, evt->data.evt_scanner_scan_report.data.data, data_size);
                      uint8_t offset = 0;
                      while (offset < data_size) {
                          if (beacon_adv_data[offset+1] != 0xFF) {      // if field not manufacturer specific data
                              offset += (beacon_adv_data[offset] + 1);  // go to next
                              continue;
                          } else {
                              if (beacon_adv_data[offset+1] > 7) { // поле достаточной длины, чтобы содержать информацию о
                                                                    // напряжении источника питания
                                  voltage[ii] =  *(int16_t*) (&beacon_adv_data[offset + 8]);
                              }
                              offset += 4;
                              app_log("data size %02x: offset=%02x: %02x%02x%02x%02x\r\n", data_size, offset, beacon_adv_data[offset+0], beacon_adv_data[offset+1], beacon_adv_data[offset+2], beacon_adv_data[offset+3]);
                              remote_temperature = *(int16_t*) (&beacon_adv_data[offset + (data[ii].offset << 1)]);
                              if (ii == 0) {
                                  sc = sl_sleeptimer_restart_timer_ms(&left_channel, data[ii].interval, &timer_stop, &left_data, 0, 0);
                              } else {
                                  sc = sl_sleeptimer_restart_timer_ms(&right_channel, data[ii].interval, &timer_stop, &right_data, 0, 0);
                              }
#ifdef HALFDEGREE
                              show_temp(((remote_temperature < 0) ? remote_temperature-25 : remote_temperature+25)/50, ii);
#elif  DEGREE
                              show_temp(((remote_temperature < 0) ? remote_temperature-50 : remote_temperature+50)/100, ii);
#endif
                              if ((device_mode & DEVICE_MODE_SAME) && (ii == 0)) {
                                  remote_temperature = *(int16_t*) (&beacon_adv_data[offset + (data[1].offset << 1)]);
#ifdef HALFDEGREE
                                  show_temp(((remote_temperature < 0) ? remote_temperature-25 : remote_temperature+25)/50, 1);
#elif  DEGREE
                                  show_temp(((remote_temperature < 0) ? remote_temperature-50 : remote_temperature+50)/100, 1);
#endif
                              }
                              app_log("Remote Temperature T=%d, timer start sc = %04x\r\n", remote_temperature, sc);
                              break;
                          }
                      }
                  } else {
                    app_log("Scanner Report: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                            evt->data.evt_scanner_scan_report.address.addr[5],
                            evt->data.evt_scanner_scan_report.address.addr[4],
                            evt->data.evt_scanner_scan_report.address.addr[3],
                            evt->data.evt_scanner_scan_report.address.addr[2],
                            evt->data.evt_scanner_scan_report.address.addr[1],
                            evt->data.evt_scanner_scan_report.address.addr[0]);
                    sc = sl_bt_scanner_stop();
                    app_log("Scanning stopped\r\n");
                    sl_sleeptimer_start_timer_ms(&connection_timeout, 20000, &not_connected_in_time, NULL, 3, 0);
//                  sl_bt_system_set_soft_timer(20ul*32768, 2, 1);
                    active_connection_num = ii;
                    sc = sl_bt_connection_open(data[ii].mac_addr,
                                               evt->data.evt_scanner_scan_report.address_type,
                                               sl_bt_gap_1m_phy, &conn_properties[active_connection_num].connection_handle); // connect to server
                    app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] sl_bt_connection open %d\n", (int)sc, ii);
                    conn_state = opening;
                  }
                  break;
              }
          }
      }
      break;

    case sl_bt_evt_connection_opened_id:
      app_log("Connection_opened_id\r\n");
      if (evt->data.evt_connection_opened.master) {
          if (conn_state == opening) {
//          sl_bt_system_set_soft_timer(0, 2, 1); // disable timer
            sl_sleeptimer_stop_timer(&connection_timeout);  //disable timer
            conn_properties[active_connection_num].connection_handle = evt->data.evt_connection_opened.connection;
            required_connections_num-- ;
            conn_state = discover_services;
            sc = sl_bt_gatt_discover_primary_services_by_uuid(conn_properties[active_connection_num].connection_handle,
                                                         sizeof(service_UUID), service_UUID);
            app_log("Start services discovery\r\n");
            app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] sl_bt_gatt_discovery primary services %d\n", (int)sc, active_connection_num);
        }
      }
      break;

    case sl_bt_evt_gatt_service_id:
      if (0 == memcmp(evt->data.evt_gatt_service.uuid.data,service_UUID, evt->data.evt_gatt_service.uuid.len)) {
        conn_properties[active_connection_num].thermometer_service_handle = evt->data.evt_gatt_service.service;
        conn_state = discover_characteristics;
        app_log("Service handle = %08x\r\n", conn_properties[active_connection_num].thermometer_service_handle);
      }
      if (evt->data.evt_gatt_service.uuid.len == 2) {
          app_log("Discovered service 16bit: %04x\r\n",
                 evt->data.evt_gatt_service.uuid.data[0] + (evt->data.evt_gatt_service.uuid.data[1] << 8));

      } else {
          app_log("Discovered service 128bit: %08x%08x%08x%08x  (%02x)\r\n",
                 evt->data.evt_gatt_service.uuid.data[12] + (evt->data.evt_gatt_service.uuid.data[13] << 8) + (evt->data.evt_gatt_service.uuid.data[14] << 16) + (evt->data.evt_gatt_service.uuid.data[15] << 24),
                 evt->data.evt_gatt_service.uuid.data[8] + (evt->data.evt_gatt_service.uuid.data[9] << 8) + (evt->data.evt_gatt_service.uuid.data[10] << 16) + (evt->data.evt_gatt_service.uuid.data[11] << 24),
                 evt->data.evt_gatt_service.uuid.data[4] + (evt->data.evt_gatt_service.uuid.data[5] << 8) + (evt->data.evt_gatt_service.uuid.data[6] << 16) + (evt->data.evt_gatt_service.uuid.data[7] << 24),
                 evt->data.evt_gatt_service.uuid.data[0] + (evt->data.evt_gatt_service.uuid.data[1] << 8) + (evt->data.evt_gatt_service.uuid.data[2] << 16) + (evt->data.evt_gatt_service.uuid.data[3] << 24),
                 evt->data.evt_gatt_service.uuid.len);
      }
      break;

    case sl_bt_evt_gatt_characteristic_id:
        if (0 == memcmp(evt->data.evt_gatt_characteristic.uuid.data, characteristic_UUID, evt->data.evt_gatt_characteristic.uuid.len)) {
            conn_properties[active_connection_num].thermometer_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;
            app_log("characteristic handle = %04x\r\n", conn_properties[active_connection_num].thermometer_characteristic_handle);
            conn_state = discovery_finished;
        }
        if (evt->data.evt_gatt_characteristic.uuid.len == 2) {
            app_log("Discovered characteristic 16bit: %04x\r\n",
               evt->data.evt_gatt_characteristic.uuid.data[0] + (evt->data.evt_gatt_characteristic.uuid.data[1] << 8));
        } else {
            app_log("Discovered characteristic 128bit: %08x%08x%08x%08x (%02x)\r\n",
                   evt->data.evt_gatt_characteristic.uuid.data[12] + (evt->data.evt_gatt_characteristic.uuid.data[13] << 8) + (evt->data.evt_gatt_characteristic.uuid.data[14] << 16) + (evt->data.evt_gatt_characteristic.uuid.data[15] << 24),
                   evt->data.evt_gatt_characteristic.uuid.data[8] + (evt->data.evt_gatt_characteristic.uuid.data[9] << 8) + (evt->data.evt_gatt_characteristic.uuid.data[10] << 16) + (evt->data.evt_gatt_characteristic.uuid.data[11] << 24),
                   evt->data.evt_gatt_characteristic.uuid.data[4] + (evt->data.evt_gatt_characteristic.uuid.data[5] << 8) + (evt->data.evt_gatt_characteristic.uuid.data[6] << 16) + (evt->data.evt_gatt_characteristic.uuid.data[7] << 24),
                   evt->data.evt_gatt_characteristic.uuid.data[0] + (evt->data.evt_gatt_characteristic.uuid.data[1] << 8) + (evt->data.evt_gatt_characteristic.uuid.data[2] << 16) + (evt->data.evt_gatt_characteristic.uuid.data[3] << 24),
                   evt->data.evt_gatt_characteristic.uuid.len);

        }
      break;

    case sl_bt_evt_gatt_procedure_completed_id:
      gatt_result = evt->data.evt_gatt_procedure_completed.result;
      app_assert(gatt_result == 0, "[E: 0x%04x] gatt_procedure_completed_id\r\n", (int)gatt_result);
      switch (conn_state) {
        case discover_services:
          sl_bt_gatt_discover_primary_services_by_uuid(conn_properties[active_connection_num].connection_handle,
                                                       sizeof(service_UUID), service_UUID);
          conn_state = not_discovered;
          break;

        case discover_characteristics:
          sc = sl_bt_gatt_discover_characteristics_by_uuid(conn_properties[active_connection_num].connection_handle,
                                                           conn_properties[active_connection_num].thermometer_service_handle,
                                                           sizeof(characteristic_UUID),
                                                           characteristic_UUID);
          conn_state = not_discovered;
          break;

        case discovery_finished:
//          sc = sl_bt_system_set_soft_timer(3 * 32768, active_connection_num, 1);
          if (active_connection_num == 0) {
              sc = sl_sleeptimer_start_timer_ms(&left_channel, 3000, &timer_callback, &left_data, 0, 0);
              app_log("Set Sleep Timer 3sec:  sc = %02x\r\n", sc);
          }
          if (active_connection_num == 1) {
              sc = sl_sleeptimer_start_timer_ms(&right_channel, 3000, &timer_callback, &right_data, 0, 0);
              app_log("Set Sleep Timer 3sec:  sc = %02x\r\n", sc);
          }

          conn_state = running;
          if ((conn_properties[0].connection_handle == CONNECTION_HANDLE_INVALID) ||
              (conn_properties[1].connection_handle == CONNECTION_HANDLE_INVALID)) {
              if (required_connections_num) {
                sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
                app_log("Scanner started  sc=%02x\r\n", sc);
              }
          }
          break;

        case not_discovered:
          sl_bt_connection_close(conn_properties[active_connection_num].connection_handle);
          break;

        default:
          app_log("sl_bt_evt_gatt_procedure_completed_id %d\r\n", conn_state);
          break;
      }
      break;

      // ----------
      // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:

      app_log("Connection_closed. Reason: %04x\r\n", evt->data.evt_connection_closed.reason);
      for (unsigned int ii = 0; ii < 2; ii++) {
          if (evt->data.evt_connection_closed.connection  == conn_properties[ii].connection_handle) {
              sl_sleeptimer_stop_timer(&connection_timeout);  //disable timer
              conn_properties[ii].connection_handle = CONNECTION_HANDLE_INVALID;
//              sl_bt_system_set_soft_timer(0, ii, 1);

              if (ii == 0) {
                  sl_sleeptimer_stop_timer  (&left_channel);
                  show_temp(ABSOLUTE_ZERO*2, 0);
              }
              if (ii == 1) {
                  sl_sleeptimer_stop_timer  (&right_channel);
                  show_temp(ABSOLUTE_ZERO*2, 1);
              }
              required_connections_num++;
              if (required_connections_num) {
                sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
                app_log("Scanner started  sc=%02x\r\n", sc);
              }
          }
      }
      break;

//    case sl_bt_evt_system_soft_timer_id:
//      timer_handle = evt->data.evt_system_soft_timer.handle;
//      if (timer_handle < 2) {
//        sc = sl_bt_gatt_read_characteristic_value(conn_properties[timer_handle].connection_handle,
//                                                  conn_properties[timer_handle].thermometer_characteristic_handle);
//        app_log("Timer Event %02x, read_value sc = %02x\r\n", timer_handle, sc);
////        if (sc == SL_STATUS_OK)  sc = sl_bt_system_set_soft_timer(data[timer_handle].interval, timer_handle, 1);
////        else                     sc = sl_bt_system_set_soft_timer( 5ul * 32768, timer_handle, 1);
////        app_log("Set Soft Timer  sc = %02x\r\n", sc);
//      } else if (timer_handle == 2) {
//
//      }
//      break;

    case sl_bt_evt_gatt_characteristic_value_id:
      if (conn_properties[0].connection_handle == evt->data.evt_gatt_characteristic_value.connection) {
          uint8_t *respData, respLen;
          int16_t remote_temperature=0;
          if (conn_properties[0].thermometer_characteristic_handle == evt->data.evt_gatt_characteristic_value.characteristic) {
            respData = evt->data.evt_gatt_characteristic_value.value.data;
            respLen = evt->data.evt_gatt_characteristic_value.value.len;
            app_log("Characteristic data Length = %02x\r\n", respLen);
            for (unsigned int ii = respLen; ii > 0; ii--) {
                remote_temperature <<= 8;
                remote_temperature |= respData[ii-1];
            }
            app_log("Remote Temperature T=0x%04x\r\n", remote_temperature);
#ifdef HALFDEGREE
            show_temp(((remote_temperature < 0) ? remote_temperature-25 : remote_temperature+25)/50, 0);
#elif  DEGREE
                show_temp(((remote_temperature < 0) ? remote_temperature-50 : remote_temperature+50)/100, 0);
#endif
            if (device_mode & DEVICE_MODE_SAME) {
#ifdef HALFDEGREE
                show_temp(((remote_temperature < 0) ? remote_temperature-25 : remote_temperature+25)/50, 1);
#elif  DEGREE
                show_temp(((remote_temperature < 0) ? remote_temperature-50 : remote_temperature+50)/100, 1);
#endif
            }
          }
      } else if (conn_properties[1].connection_handle == evt->data.evt_gatt_characteristic_value.connection) {
          uint8_t *respData, respLen;
          int16_t remote_temperature=0;
          if (conn_properties[1].thermometer_characteristic_handle == evt->data.evt_gatt_characteristic_value.characteristic) {
            respData = evt->data.evt_gatt_characteristic_value.value.data;
            respLen = evt->data.evt_gatt_characteristic_value.value.len;
            for (unsigned int ii = respLen; ii > 0; ii--) {
                remote_temperature <<= 8;
                remote_temperature |= respData[ii-1];
            }
#ifdef HALFDEGREE
            show_temp(((remote_temperature < 0) ? remote_temperature-25 : remote_temperature+25)/50, 1);
#elif  DEGREE
            show_temp(((remote_temperature < 0) ? remote_temperature-50 : remote_temperature+50)/100, 1);
#endif
         }
      }
      break;

    default:
      break;
  }
}

void timer_callback(sl_sleeptimer_timer_handle_t *handle, void *timer_data) {
  sl_status_t sc;
  uint32_t tick;

  sc = sl_bt_gatt_read_characteristic_value(conn_properties[*(uint32_t*)timer_data].connection_handle,
                                            conn_properties[*(uint32_t*)timer_data].thermometer_characteristic_handle);
  app_log("Timer Event %02x, read_value sc = %02x\r\n", *(uint32_t*)timer_data, sc);
  if (sc == SL_STATUS_OK) sc = sl_sleeptimer_ms32_to_tick(data[*(uint32_t*)timer_data].interval, &tick );
  else                    sc = sl_sleeptimer_ms32_to_tick(5000, &tick );
  if (sc == SL_STATUS_INVALID_PARAMETER) tick = sl_sleeptimer_get_max_ms32_conversion();

  sc = sl_sleeptimer_start_timer(handle, tick, &timer_callback, timer_data, 0, 0);
  app_log("Set Sleep Timer tick = %10d sc = %02x\r\n", tick, sc);
}

void not_connected_in_time(sl_sleeptimer_timer_handle_t *handle, void *timer_data) {
  sl_status_t sc;
  (void) handle;
  (void) timer_data;

  // connection timeout
  if (conn_properties[active_connection_num].connection_handle != CONNECTION_HANDLE_INVALID) {
      sl_bt_connection_close(conn_properties[active_connection_num].connection_handle);
  } else {
    conn_properties[active_connection_num].connection_handle = CONNECTION_HANDLE_INVALID;
    required_connections_num++;
    sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
    app_log("Connection timeout. Scanner started  sc=%02x\r\n", sc);
  }
}
void timer_stop(sl_sleeptimer_timer_handle_t *handle, void *timer_data) {

  (void) handle;
  if (*(uint32_t*)timer_data == 0) {
      show_temp(ABSOLUTE_ZERO*2, 0);
      voltage[0] = 5;
      if (device_mode & DEVICE_MODE_SAME) show_temp(ABSOLUTE_ZERO*2, 1);
  }
  if (*(uint32_t*)timer_data == 1) {
      show_temp(ABSOLUTE_ZERO*2, 1);
      voltage[1] = 5;
  }
}

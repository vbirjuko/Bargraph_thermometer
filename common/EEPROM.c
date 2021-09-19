/*
 * EEPROM.c
 *
 *  Created on: 7 мая 2021 г.
 *      Author: Владимир Бирюков.
 */

/*
 * Функции работы с конфигурацией: чтение и сохранение в EEPROM
 * и обработка событий BT стека с чтением и записью конфигурации.
 *
 * Если определён USE_NVM - конфигурация сохраняется в эмулированном EEPROM
 * иначе в EEPROM типа 24c02-24c16 подключенной через i2c.
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
#define USE_NVM

#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "em_gpio.h"
#include "app_assert.h"
#include "app_log.h"
#include "EEPROM.h"
#include "sx_memcpy.h"
#ifndef USE_NVM
#include "i2c_drv.h"
#else
#define LEFT_KEY  (0x4007)
#define RIGHT_KEY (0x4008)
#endif

struct server_addr_t data[2], scratchpad[2];
unsigned int eeprom_update_flag = 0;
uint16_t data_sent_size;

void read_eeprom_config(unsigned int start) {

#ifdef USE_NVM
  sl_status_t nvm_result;
  size_t bytes_read;

  nvm_result = sl_bt_nvm_load(LEFT_KEY, sizeof(scratchpad[0]), &bytes_read, (uint8_t*)&scratchpad[0]);
  if (nvm_result != SL_STATUS_OK) start = 0;
  app_log("NVM load key = 0x%04x rc=%2d, bytes read = %2d\r\n", LEFT_KEY, nvm_result, bytes_read);
  nvm_result = sl_bt_nvm_load(RIGHT_KEY, sizeof(scratchpad[1]), &bytes_read, (uint8_t*)&scratchpad[1]);
  if (nvm_result != SL_STATUS_OK) start = 0;
  app_log("NVM load key = 0x%04x rc=%2d, bytes read = %2d\r\n", RIGHT_KEY, nvm_result, bytes_read);
#else
  t_i2c_status i2c_result;

  i2c_result = i2c_wr_reg(EEPROM_ADDR, 0x30, (uint8_t*)&scratchpad[1], 0); // dummy access to synchronize EEPROM

  i2c_result = i2c_rd_reg(EEPROM_ADDR, 0x00, (uint8_t*)&scratchpad[0], sizeof(scratchpad[0]));
  app_log("EEPROM Read 0x00 ErrorCode %02x\r\n", i2c_result);
#ifdef DEBUG_I2C
  app_log(debug_str);
#endif

  i2c_result = i2c_rd_reg(EEPROM_ADDR, 0x10, (uint8_t*)&scratchpad[1], sizeof(scratchpad[1]));
  app_log("EEPROM Read 0x10 ErrorCode %02x\r\n", i2c_result);
#ifdef DEBUG_I2C
  app_log(debug_str);
#endif
#endif
  app_log("Sensor 0:\r\nData.MAC %02x:%02x:%02x:%02x:%02x:%02x\r\n", scratchpad[0].mac_addr.addr[5], scratchpad[0].mac_addr.addr[4], scratchpad[0].mac_addr.addr[3],
          scratchpad[0].mac_addr.addr[2], scratchpad[0].mac_addr.addr[1], scratchpad[0].mac_addr.addr[0]);
  app_log("data.interval = %d\r\n", scratchpad[0].interval);

  app_log("Sensor 1:\r\nData.MAC %02x:%02x:%02x:%02x:%02x:%02x\r\n", scratchpad[1].mac_addr.addr[5], scratchpad[1].mac_addr.addr[4], scratchpad[1].mac_addr.addr[3],
          scratchpad[1].mac_addr.addr[2], scratchpad[1].mac_addr.addr[1], scratchpad[1].mac_addr.addr[0]);
  app_log("data.interval = %d\r\n", scratchpad[1].interval);

  if (start) {
    data[0] = scratchpad[0];
    data[1] = scratchpad[1];
  }
}

void write_eeprom_config(unsigned int parts) {
#ifdef USE_NVM
  sl_status_t nvm_result;
  if (parts & (1ul << 0)) {
    nvm_result = sl_bt_nvm_save(LEFT_KEY, sizeof(scratchpad[0]), (uint8_t*)&scratchpad[0]);
    app_log("NVM save key=0x%04x rc=0x%08x\r\n", LEFT_KEY, nvm_result);
    parts &= ~(1ul << 0);
  }
  if (parts & (1ul << 1)) {
    nvm_result = sl_bt_nvm_save(RIGHT_KEY, sizeof(scratchpad[1]), (uint8_t*)&scratchpad[1]);
    app_log("NVM save key=0x%04x rc=0x%08x\r\n", RIGHT_KEY, nvm_result);
    parts &= ~(1ul << 1);
  }
#else
  t_i2c_status i2c_result;

  while (parts) {
    if (parts & (1ul << 0)) {
      i2c_result = i2c_wr_reg(EEPROM_ADDR, 0x00, (uint8_t*)&scratchpad[0], sizeof(scratchpad[0]));
      app_log("Try write 0x00 result %02x\r\n", i2c_result);
      if (i2c_result == I2C_SUCCESS) {
          parts &= ~(1ul << 0);
      }
    }
    if (parts & (1ul << 1)) {
      i2c_result = i2c_wr_reg(EEPROM_ADDR, 0x10, (uint8_t*)&scratchpad[1], sizeof(scratchpad[1]));
      app_log("Try write 0x10 result %02x\r\n", i2c_result);
      if (i2c_result == I2C_SUCCESS) {
          parts &= ~(1ul << 1);
      }
    }
  }
#endif
}



void eeprom_on_event(sl_bt_msg_t *evt) {
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_bt_evt_gatt_server_user_write_request_id:
      app_log("write characteristic req\r\n");
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_left_sensor) {
          uint8_t data_size = evt->data.evt_gatt_server_user_write_request.value.len;
          if (data_size > sizeof(scratchpad[0])) data_size = sizeof(scratchpad[0]);
          memcpy_array(&scratchpad[0], evt->data.evt_gatt_server_user_write_request.value.data, data_size);
          eeprom_update_flag = (1ul << 0);
          sl_bt_gatt_server_send_user_write_response(
              evt->data.evt_gatt_server_user_write_request.connection,
              gattdb_left_sensor, SL_STATUS_OK);
      }
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_right_sensor) {
          uint8_t data_size = evt->data.evt_gatt_server_user_write_request.value.len;
          if (data_size > sizeof(scratchpad[1])) data_size = sizeof(scratchpad[1]);
          memcpy_array(&scratchpad[1], evt->data.evt_gatt_server_user_write_request.value.data, data_size);
          eeprom_update_flag = (1ul << 1);
          sl_bt_gatt_server_send_user_write_response(
              evt->data.evt_gatt_server_user_write_request.connection,
              gattdb_right_sensor, SL_STATUS_OK);
      }
      break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      app_log("read characteristic req\r\n");
      if (gattdb_left_sensor == evt->data.evt_gatt_server_user_read_request.characteristic) {
          sc = sl_bt_gatt_server_send_user_read_response(
              evt->data.evt_gatt_server_user_read_request.connection,
              evt->data.evt_gatt_server_user_read_request.characteristic,
              0,
              sizeof(scratchpad[0]),
              (uint8_t*)&scratchpad[0],
              &data_sent_size);
          app_assert(sc == SL_STATUS_OK, "Read Req 0x00, sc=%08x\r\n", sc);
      }
      if (gattdb_right_sensor == evt->data.evt_gatt_server_user_read_request.characteristic) {
          sc = sl_bt_gatt_server_send_user_read_response(
              evt->data.evt_gatt_server_user_read_request.connection,
              evt->data.evt_gatt_server_user_read_request.characteristic,
              0,
              sizeof(scratchpad[1]),
              (uint8_t*)&scratchpad[1],
              &data_sent_size);
          app_assert(sc == SL_STATUS_OK, "Read Req 0x10, sc=%08x\r\n", sc);
      }
      break;
  }// switch
}

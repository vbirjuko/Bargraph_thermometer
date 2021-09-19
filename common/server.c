/*
 * server.c
 *
 *  Created on: 2 апр 2021 г.
 *      Author: Владимир Бирюков.
 *
 *
 * Модуль обеспечивает инициализацию модулей, запуск сервера при нажатой кнопке
 * и главный цикл программы.
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
#include "em_common.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#ifdef OLED_I2C
#include "i2c_drv.h"
#include "display.h"
#endif
#include "configuration.h"
#include "Timer_scan.h"
#include "local_temp.h"
#include "EEPROM.h"
#include "client.h"
#include "server.h"
#include "visualise.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
unsigned int device_mode = (DEVICE_MODE_LOCAL_LEFT | DEVICE_MODE_LOCAL_RIGHT);

void reinit_bt_address(void) {
  unsigned int ii, jj;
  device_mode = (DEVICE_MODE_LOCAL_LEFT | DEVICE_MODE_LOCAL_RIGHT);
  jj = 0;
  for (ii = 0; ii < 6; ii++){
      if (data[0].mac_addr.addr[ii]) device_mode &= ~(DEVICE_MODE_LOCAL_LEFT);
      if (data[1].mac_addr.addr[ii]) device_mode &= ~(DEVICE_MODE_LOCAL_RIGHT);
      jj |= (data[0].mac_addr.addr[ii] | data[1].mac_addr.addr[ii]);
  }
  for (ii = 0; ii < 6; ii++){
      if (data[0].mac_addr.addr[ii] != data[1].mac_addr.addr[ii]) break;
  }
  if (ii == 6)  device_mode |= DEVICE_MODE_SAME;
  if (jj)       device_mode |= DEVICE_MODE_REMOTE;

  if (device_mode & DEVICE_MODE_REMOTE) {
      required_connections_num = 2;
      if (device_mode & DEVICE_MODE_SAME) required_connections_num = 1;
  } else {
      required_connections_num = 0;
  }
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////


  CMU_ClockEnable(cmuClock_GPIO, 1);
  GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 0);
  GPIO_PinModeSet(KEY_PORT, KEY_PIN, gpioModeInputPull, 1);
  Timer_Init();
#ifdef OLED_I2C
  i2c_master_init();
#endif
  read_eeprom_config(1);
#ifdef OLED_I2C
  display_init();
  prepare_screen();
#endif
  reinit_bt_address();
}


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
  static unsigned int state = 0, reread_eeprom = 0;

  if (scan_flag) {
    scan_flag = 0;

    if (state) {
        unsigned int local_temp;
        local_temp=get_local_temp();
        if (device_mode & DEVICE_MODE_LOCAL_LEFT)  {  show_temp(local_temp, 0); }
        if (device_mode & DEVICE_MODE_LOCAL_RIGHT) {  show_temp(local_temp, 1); }
        state = 0;
    } else {
        request_local_temp(64);
        state = 1;
    }
    if (eeprom_update_flag) {
        write_eeprom_config(eeprom_update_flag);
        eeprom_update_flag = 0;
        reread_eeprom = 3;
    }
    if (reread_eeprom) {
        if (--reread_eeprom == 0) {
            read_eeprom_config(0);
        }
    }
    visualise_iterate();
  }
}

bool app_is_ok_to_sleep(void)
{
  return false;
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void server_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];


  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      sl_bt_sm_configure(0x07,sm_io_capability_keyboardonly);  // allow all connections
      sl_bt_sm_set_bondable_mode(1);                // enable new bondings

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
                            advertising_set_handle,
                            160, // min. adv. interval (milliseconds * 1.6)
                            160, // max. adv. interval (milliseconds * 1.6)
                            0,   // adv. duration
                            0);  // max. num. adv. events
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);

      // Do not start advertise if button not pressed.
      if (GPIO_PinInGet(KEY_PORT, KEY_PIN) == 0) {

        sl_bt_sm_delete_bondings();               // if BUTTON1 is pressed
        app_log("All bondings are erased.\r\n");

        // Start general advertising and enable connections.
        sc = sl_bt_advertiser_start(
                            advertising_set_handle,
                            advertiser_general_discoverable,
                            advertiser_connectable_scannable);
        app_assert(sc == SL_STATUS_OK,
                      "[E: 0x%04x] Failed to start advertising\n",
                      (int)sc);
      }
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

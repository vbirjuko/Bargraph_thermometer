/*
 * EEPROM.h
 *
 *  Created on: 7 мая 2021 г.
 *      Author: Владимир Бирюков
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "sl_bluetooth.h"

#define EEPROM_ADDR (0xA0)

typedef struct server_addr_t {
  uint32_t interval;   // 4
  bd_addr mac_addr;   // 6
  uint32_t offset;
}server_addr;

extern struct server_addr_t data[2];
extern unsigned int eeprom_update_flag;

void read_eeprom_config(unsigned int start);
void write_eeprom_config(unsigned int parts);
void eeprom_on_event(sl_bt_msg_t *evt);

#endif /* EEPROM_H_ */

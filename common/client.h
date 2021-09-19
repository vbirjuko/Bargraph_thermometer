/*
 * client.h
 *
 *  Created on: 25 мая 2021 г.
 *   Author: Владимир Бирюков
 */

#ifndef CLIENT_H_
#define CLIENT_H_

extern uint8_t   required_connections_num;

void client_on_event(sl_bt_msg_t *evt);

#endif /* CLIENT_H_ */

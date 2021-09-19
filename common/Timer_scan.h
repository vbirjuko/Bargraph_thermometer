/*
 * Timer_scan.h
 *
 *  Created on: 4 апр. 2021 г.
 *      Author: Владимир Бирюков.
 */

#ifndef TIMER_SCAN_H_
#define TIMER_SCAN_H_

extern volatile unsigned int scan_flag;
void Timer_Init(void);
void show_left_scale(unsigned int element_count);
void show_right_scale(unsigned int element_count);

#endif /* TIMER_SCAN_H_ */

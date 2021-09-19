/*
 * visualise.c
 *
 *  Created on: 29 июн. 2021 г.
 *      Author: Владимир Бирюков.
 */
/*
 * Модуль визуализации показаний. Если показания изменились по сравнению с
 * предыдущей итерацией, то показания шкалы плавно будут изменяться в
 * соответствующем направлении. Итерация соответсвует одному кадру развертки
 * шкалы.
 *
 * При желании сюда можно добавить screensaver.
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
#ifdef OLED_I2C
#include "display.h"
#endif
#include "EEPROM.h"
#include "Timer_scan.h"
#include "visualise.h"
#include "configuration.h"

int scale_offsets[] = {T_SCALE_OFFSET, RH_SCALE_OFFSET, 0};

int temp_left = ABSOLUTE_ZERO*2, temp_right = ABSOLUTE_ZERO*2;

void show_temp (int temp, unsigned int right) {
  if (right) temp_right = temp;
  else        temp_left = temp;
}

void visualise_iterate(void) {
  static unsigned int position_left = 0, position_right = 0;
  static unsigned int screensaver_timer = 70*SCREENSAVER_INTERVAL, screensaver = 0;
  static unsigned int direction = 0;
  unsigned int new_position_left, new_position_right;

  if (temp_left < -scale_offsets[data[0].offset])  new_position_left = 0;
  else                                             new_position_left = temp_left + scale_offsets[data[0].offset];

  if (temp_right < -scale_offsets[data[1].offset]) new_position_right = 0;
  else                                             new_position_right = temp_right + scale_offsets[data[1].offset];


  if (screensaver_timer) {
      screensaver_timer--;
      if (position_left < new_position_left) position_left++;
      if (position_left > new_position_left) position_left--;

      if (position_right < new_position_right)  position_right++;
      if (position_right > new_position_right)  position_right--;
  } else {  // Screensaver
      if (screensaver++ < 70*SCREENSAVER_LENGTH) {
          if (direction) {
              if (position_right) position_right--;
              position_left++;
              if (position_right == 0 && position_left > 200) direction = 0;
          } else {
              if (position_left) position_left--;
              position_right++;
              if (position_left == 0 && position_right > 200) direction = 1;
          }
      } else {
          screensaver_timer = 70*SCREENSAVER_INTERVAL;
          screensaver = 0;
      }
  }
#ifdef OLED_I2C
  show_scale(position_left, 0);
  show_scale(position_right, 1);
  update_display();
#endif
  show_left_scale(position_left);
  show_right_scale(position_right);
}

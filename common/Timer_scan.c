 /*
 * Timer_scan.c
 *
 *  Created on: 3 апр. 2021 г.
 *      Author: Владимир Бирюков.
 */
/*
 * Модуль обслуживающий дискретный шкальный индикатор с самосканированием.
 * Трёхфазную последовательность генерят TIMER1 TIMER3 - синхронно.
 * ШИМ выводится на выводы, а затем через PRS преобразуется и выводится
 * на другие выводы.
 * Для управления анодами TIMER2 - левый, TIMER4 - правый.
 *
 * Состоит из длинной инициализации и одного обработчика прерывания. Ну и пары
 * функций записывающих в регистры сравнения сколько элементов шкалы отобразить.
 */

#define SMOOTH
/*
 * В исходной версии анодные таймеры тактировались от катодных так, что число
 * загруженное в регистр сравнения совпадало с числом светящихся катодов. Но так
 * как реакция на выключение анодного напряжения происходит после переключения
 * катодов, наличествует нежелательная подсветка следующего за последним активным
 * сегментом. Поэтому в этом варианте таймер анодов тактируется от того же
 * генератора что и катодные, только с частотой в 8 раз меньше, чтобы уложить
 * весь период в 16 бит. В регистр сравнения анодного таймера надо теперь
 * загружать не число сегментов, а время, сколько должна светиться шкала от
 * начала развёртки. Таблица этих интервалов создаётся при иициализации. При этом
 * функция загружающая регистр сравнения отнимает небольшую величину, чтобы
 * анодное напряжение снималось за 5 микросекунд до переключения катодов.
 * И так как вход тактирования анодного таймера по входу захвата больше
 * не используется, нет надобности в двух таймерах.
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "configuration.h"

// 70us => 38.4/2 * 70 = 1344
#ifndef IN34
#define t70us  1344
#define t140us 2688
#else
#define t70us  1980
#define t140us 1980
#endif

#ifdef DISABLE_HILIGHT
#define t110us  t70us
#define t130us  t70us
#else
#define t110us  2178
#define t130us  2496
#endif
volatile unsigned int scan_flag;


#define ROWFRAME(X, Y, Z, P, E) {X, X+Y, X+Y+Z, X+Y+Z+P, E}
uint16_t sequence[][5] = {
//    {t140us, t140us+t70us, t140us+t70us+t70us, t140us+t70us+t70us+t70us},
    ROWFRAME(t140us, t70us, t70us, t70us, 2),       // 0, 1, 2, 3
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 4, 5, 6   1
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 7, 8, 9   2
    ROWFRAME(0, t130us, t70us, t70us, 0),           // 10, 11, 12  3
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 13, 14, 15  4
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 16, 17, 18  5
    ROWFRAME(0, t70us, t110us, t70us, 0),           // 19, 20, 21  6
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 22, 23, 24  7
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 25, 26, 27  8
    ROWFRAME(0, t70us, t70us, t130us, 0),           // 28, 29, 30  9
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 31, 32, 33  10
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 34, 35, 36  11
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 37, 38, 39  12
    ROWFRAME(0, t110us, t70us, t70us, 0),           // 40, 41, 42  13
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 43, 44, 45, 14
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 46, 47, 48  15
    ROWFRAME(0, t70us, t130us, t70us, 0),           // 49, 50, 51  16
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 52, 53, 54  17
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 55, 56, 57  18
    ROWFRAME(0, t70us, t70us, t110us, 0),           // 58, 59, 60  19
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 61, 62, 63  20
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 64, 65, 66  21
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 67, 68, 69  22
    ROWFRAME(0, t130us, t70us, t70us, 0),           // 70, 71, 72  23
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 73, 74, 75  24
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 76, 77, 78  25
    ROWFRAME(0, t70us, t110us, t70us, 0),           // 79, 80, 81  26
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 82, 83, 84  27
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 85, 86, 87  28
    ROWFRAME(0, t70us, t70us, t130us, 0),           // 88, 89, 90  29
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 91, 92, 93  30
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 94, 95, 96  31
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 97, 98, 99  32
    ROWFRAME(0, t110us, t70us, t70us, 0),           // 100, 101, 102 33
#ifndef IN34
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 103, 104, 105 34
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 106, 107, 108 35
    ROWFRAME(0, t70us, t130us, t70us, 0),           // 109, 110, 111 36
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 112, 113, 114 37
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 115, 116, 117 38
    ROWFRAME(0, t70us, t70us, t110us, 0),           // 118, 119, 120 39
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 121, 122, 123 40
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 124, 125, 126 41
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 127, 128, 129 42
    ROWFRAME(0, t130us, t70us, t70us, 0),           // 130, 131, 132 43
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 133, 134, 135 44
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 136, 137, 138 45
    ROWFRAME(0, t70us, t110us, t70us, 0),           // 139, 140, 141 46
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 142, 143, 144 47
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 145, 146, 147 48
    ROWFRAME(0, t70us, t70us, t130us, 0),           // 148, 149, 150 49
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 151, 152, 153 50
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 154, 155, 156 51
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 157, 158, 159 52
    ROWFRAME(0, t110us, t70us, t70us, 0),           // 160, 161, 162 53
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 163, 164, 165 54
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 166, 167, 168 55
    ROWFRAME(0, t70us, t130us, t70us, 0),           // 169, 170, 171 56
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 172, 173, 174 57
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 175, 176, 177 58
    ROWFRAME(0, t70us, t70us, t110us, 0),           // 178, 179, 180 59
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 181, 182, 183 60
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 184, 185, 186 61
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 187, 188, 189 62
    ROWFRAME(0, t130us, t70us, t70us, 0),           // 190, 191, 192 63
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 193, 194, 195 64
    ROWFRAME(0, t70us, t70us, t70us, 0),            // 196, 197, 198 65
    ROWFRAME(0, t70us, t110us, 5, 0),               // 199, 200, 201 66
#endif
};
#define SEQUENCE_LINES  (sizeof(sequence)/sizeof(sequence[0]))

#ifdef SMOOTH
uint16_t channel_durations[SEQUENCE_LINES * 4], ch_durations_last = 0;
#endif

void TIMER1_IRQHandler(void) {
  static unsigned int line = SEQUENCE_LINES-1;
  if (TIMER1->IF & TIMER_IF_OF) {
    TIMER1->IF_CLR = TIMER_IF_OF;
    TIMER1->CC[0].OCB = sequence[line][0];
    TIMER1->CC[1].OCB = sequence[line][1];
    TIMER1->CC[2].OCB = sequence[line][2];
    TIMER3->CC[0].OCB = sequence[line][4];
    TIMER1->TOPB = sequence[line][3];
    TIMER3->TOPB = sequence[line][3];
    if (++line >= SEQUENCE_LINES) {
        line = 0;
        scan_flag = 1;
    }
  }
}


void Timer_Init(void) {
  uint32_t current_duration = 0;
  uint16_t *duration_ptr = channel_durations;
  for (unsigned int ii = 0; ii < (SEQUENCE_LINES * 4); ii++) {
      if (sequence[ii / 4][ii % 4]) {
          *duration_ptr++ = (current_duration + sequence[ii / 4][ii % 4]) / 8;
          ch_durations_last++;
      }
      if ((ii % 4) == 3) current_duration += sequence[ii / 4][3];
  }

  CMU_ClockEnable(cmuClock_GPIO, 1);
  CMU_ClockEnable(cmuClock_PRS, 1);
  CMU_ClockEnable(cmuClock_TIMER1, 1);
  CMU_ClockEnable(cmuClock_TIMER2, 1);
  CMU_ClockEnable(cmuClock_TIMER3, 1);
#ifndef SMOOTH
  CMU_ClockEnable(cmuClock_TIMER4, 1);
#endif

  GPIO_PinModeSet(gpioPortB, 0, gpioModePushPull, 0); // анод левого канала
  GPIO_PinModeSet(gpioPortB, 1, gpioModePushPull, 0); // анод правого канала
  GPIO_PinModeSet(gpioPortB, 2, gpioModePushPull, 0); // катод 1 и 2 фазы
  GPIO_PinModeSet(gpioPortB, 3, gpioModePushPull, 0); // катод сброса
  GPIO_PinModeSet(gpioPortB, 4, gpioModePushPull, 0); // катод 3 фазы
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0); // катод 1 фазы
  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0); // катод 2 фазы

#ifndef SMOOTH
  // *********************************************************************
  // Конфигурация счетчиков, управляющих анодами индикатора
  // Timer2 - левый, Timer4 - правый. СС0 - у обоих сконфигурирован как сброс,
  // CC1 - вход счетных импульсов, CC2 - выход ШИМ
  // *********************************************************************

  TIMER2->CFG = TIMER_CFG_MODE_UP | TIMER_CFG_SYNC_DISABLE |
                TIMER_CFG_DISSYNCOUT_DIS | TIMER_CFG_DEBUGRUN |
                TIMER_CFG_CLKSEL_CC1;
  TIMER2->CC[0].CFG = TIMER_CC_CFG_MODE_INPUTCAPTURE |
                      TIMER_CC_CFG_INSEL_PRSASYNCLEVEL ;
  TIMER2->CC[1].CFG = TIMER_CC_CFG_MODE_INPUTCAPTURE |
                      TIMER_CC_CFG_INSEL_PRSASYNCLEVEL;
  TIMER2->CC[2].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;

  TIMER2->EN = TIMER_EN_EN;
  TIMER2->TOP = SEQUENCE_LINES*3;
  TIMER2->CTRL = TIMER_CTRL_RISEA_RELOADSTART;
  TIMER2->CC[1].CTRL = TIMER_CC_CTRL_ICEDGE_FALLING;
  TIMER2->CC[2].OC = 10;

  TIMER4->CFG = TIMER_CFG_MODE_UP | TIMER_CFG_SYNC_DISABLE |
                TIMER_CFG_DISSYNCOUT_DIS | TIMER_CFG_DEBUGRUN |
                TIMER_CFG_CLKSEL_CC1;
  TIMER4->CC[0].CFG = TIMER_CC_CFG_MODE_INPUTCAPTURE |
                      TIMER_CC_CFG_INSEL_PRSASYNCLEVEL ;
  TIMER4->CC[1].CFG = TIMER_CC_CFG_MODE_INPUTCAPTURE |
                      TIMER_CC_CFG_INSEL_PRSASYNCLEVEL;
  TIMER4->CC[2].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;

  TIMER4->EN = TIMER_EN_EN;
  TIMER4->TOP = SEQUENCE_LINES*3;
  TIMER4->CTRL = TIMER_CTRL_RISEA_RELOADSTART;
  TIMER4->CC[1].CTRL = TIMER_CC_CTRL_ICEDGE_FALLING;
#else
  // *********************************************************************
  // Конфигурация счетчика, управляющего анодами индикатора
  // СС0 - сконфигурирован как сброс,
  // CC1 - выход ШИМ левый, CC2 - выход ШИМ правый
  // *********************************************************************
  TIMER2->CFG = TIMER_CFG_MODE_UP | TIMER_CFG_SYNC_DISABLE |
                TIMER_CFG_DISSYNCOUT_DIS | TIMER_CFG_DEBUGRUN |
                TIMER_CFG_CLKSEL_PRESCEM01GRPACLK | TIMER_CFG_PRESC_DIV16;
  TIMER2->CC[0].CFG = TIMER_CC_CFG_MODE_INPUTCAPTURE |
                      TIMER_CC_CFG_INSEL_PRSASYNCLEVEL ;
  TIMER2->CC[1].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;
  TIMER2->CC[2].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;

  TIMER2->EN = TIMER_EN_EN;
  TIMER2->TOP =  *(--duration_ptr) ;
  TIMER2->CTRL = TIMER_CTRL_RISEA_RELOADSTART;
#endif

  // *********************************************************************
  // Генератор последовательности фаз для катодов
  // *********************************************************************

  TIMER1->CFG = TIMER_CFG_PRESC_DIV2 | TIMER_CFG_MODE_UP |
                TIMER_CFG_SYNC_DISABLE | TIMER_CFG_DISSYNCOUT_EN |
                TIMER_CFG_DEBUGRUN;
#ifndef SMOOTH
  TIMER1->CC[0].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_PULSE;
  TIMER1->CC[1].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_PULSE;
  TIMER1->CC[2].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_PULSE;
#else
  TIMER1->CC[0].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;
  TIMER1->CC[1].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;
  TIMER1->CC[2].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;
#endif
  TIMER1->EN = TIMER_EN_EN;
  TIMER1->TOP = sequence[0][3];
  TIMER1->TOPB = sequence[1][3];
  TIMER1->CNT = 0x0;
  TIMER1->IEN = TIMER_IEN_OF;

  TIMER1->CC[0].CTRL = TIMER_CC_CTRL_OUTINV_DEFAULT;
  TIMER1->CC[1].CTRL = TIMER_CC_CTRL_OUTINV;
  TIMER1->CC[2].CTRL = TIMER_CC_CTRL_OUTINV;

  TIMER1->CC[0].OC = sequence[0][0];
  TIMER1->CC[1].OC = sequence[0][1];
  TIMER1->CC[2].OC = sequence[0][2];

  TIMER1->CC[0].OCB = sequence[1][0];
  TIMER1->CC[1].OCB = sequence[1][1];
  TIMER1->CC[2].OCB = sequence[1][2];

  TIMER3->CFG = TIMER_CFG_PRESC_DIV2 | TIMER_CFG_MODE_UP |
                TIMER_CFG_SYNC_ENABLE | TIMER_CFG_DISSYNCOUT_DIS |
                TIMER_CFG_DEBUGRUN;
  TIMER3->CC[0].CFG = TIMER_CC_CFG_MODE_PWM | TIMER_CC_CFG_PRSCONF_LEVEL;
  TIMER3->EN = TIMER_EN_EN;
  TIMER3->TOP = sequence[0][3];
  TIMER3->TOPB = sequence[1][3];
  TIMER3->CNT = 0;
  TIMER3->CC[0].OC = sequence[0][4];
  TIMER3->CC[0].OCB = sequence[1][4];

  GPIO->TIMERROUTE[1].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN |
                                GPIO_TIMER_ROUTEEN_CC1PEN |
                                GPIO_TIMER_ROUTEEN_CC2PEN ;

  GPIO->TIMERROUTE[1].CC0ROUTE   = gpioPortB | (3ul << 16);
  GPIO->TIMERROUTE[1].CC1ROUTE   = gpioPortB | (2ul << 16);
  GPIO->TIMERROUTE[1].CC2ROUTE   = gpioPortB | (4ul << 16);

  NVIC_SetPriority(TIMER1_IRQn, 2);
  NVIC_EnableIRQ(TIMER1_IRQn);


  // *********************************************************************
  // Соединяем ведущий таймер 1 и 3 с ведомыми 2 и 4:
  // PRS канал 2 - сигнал CC0 Timer3
  // PRS Канал 1 - логическое ИЛИ канала 2 и CC1
  // PRS Канал 8 - логическое ИЛИ канала 1 и CC2
  // ===============================================
  // PRS Канал 3 - Логическое ИЛИ канала 8 и CC0 таймера 1
  // Теперь тут собраны все сигналы по лог.ИЛИ для подачи счетных импульсов
  // на CC1 таймеров 2 и 4
  // *********************************************************************

  PRS->ASYNC_CH[2].CTRL = PRS_ASYNC_CH_CTRL_AUXSEL_DEFAULT |
                          PRS_ASYNC_CH_CTRL_FNSEL_A |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC0;

#ifndef SMOOTH
  PRS->ASYNC_CH[1].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A_OR_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC1 |
                          (0x00002UL << 24);

  PRS->ASYNC_CH[8].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A_OR_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC2 |
                          (0x00001UL << 24);

  PRS->ASYNC_CH[3].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A_OR_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC0 |
                          (0x00008UL << 24) ;
#endif
  // *********************************************************************
  // Формирование выходных сигналов выбора катодов.
  // Входные сигналы: PWM выходы Timer1 на выводах порта B 2 - 4
  // Канал 5:  повторяет сигнал порта B2 (PWM Timer1CC1)
  // Канал 10: ИЛИ-НЕ Timer1CC0 и пятого канала (PWM Timer1CC1)
  // Канал 11: ИЛИ-НЕ порта B1 (PWM Timer1CC2) и инверсии пятого канала (PWM Timer1CC1)
  // *********************************************************************
#ifndef SMOOTH
  GPIO_ExtIntConfig(gpioPortB, 2, 2, 0, 0, false);
  GPIO_ExtIntConfig(gpioPortB, 3, 3, 0, 0, false);
  GPIO_ExtIntConfig(gpioPortB, 4, 4, 0, 0, false);

  PRS->ASYNC_CH[5].CTRL = PRS_ASYNC_CH_CTRL_AUXSEL_DEFAULT |
                          PRS_ASYNC_CH_CTRL_FNSEL_A |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO |
                          PRS_ASYNC_CH_CTRL_SIGSEL_GPIOPIN2;

  PRS->ASYNC_CH[10].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A_NOR_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO |
                          PRS_ASYNC_CH_CTRL_SIGSEL_GPIOPIN3 |
                          (0x00005UL << 24);

  PRS->ASYNC_CH[11].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_NOT_A_AND_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_GPIO |
                          PRS_ASYNC_CH_CTRL_SIGSEL_GPIOPIN4 |
                          (0x00005UL << 24);
#else
  PRS->ASYNC_CH[5].CTRL = PRS_ASYNC_CH_CTRL_AUXSEL_DEFAULT |
                          PRS_ASYNC_CH_CTRL_FNSEL_A |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC1;

  PRS->ASYNC_CH[10].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A_NOR_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC0 |
                          (0x00005UL << 24);

  PRS->ASYNC_CH[11].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_NOT_A_AND_B |
                          PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1 |
                          PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC2 |
                          (0x00005UL << 24);
#endif
  // выводим сигналы на контакты:
#ifndef SMOOTH
  GPIO->TIMERROUTE[2].CC2ROUTE   = gpioPortB | (0ul << 16);
  GPIO->TIMERROUTE[4].CC2ROUTE   = gpioPortB | (1ul << 16);
  GPIO->TIMERROUTE[2].ROUTEEN = GPIO_TIMER_ROUTEEN_CC2PEN;
  GPIO->TIMERROUTE[4].ROUTEEN = GPIO_TIMER_ROUTEEN_CC2PEN;
#else
  GPIO->TIMERROUTE[2].CC1ROUTE   = gpioPortB | (0ul << 16);
  GPIO->TIMERROUTE[2].CC2ROUTE   = gpioPortB | (1ul << 16);
  GPIO->TIMERROUTE[2].ROUTEEN = GPIO_TIMER_ROUTEEN_CC1PEN |
                                GPIO_TIMER_ROUTEEN_CC2PEN;
#endif

  GPIO->PRSROUTE[0].ASYNCH10ROUTE   = gpioPortC | (0ul << 16);
  GPIO->PRSROUTE[0].ASYNCH11ROUTE   = gpioPortC | (1ul << 16);
  GPIO->PRSROUTE[0].ROUTEEN = GPIO_PRS_ROUTEEN_ASYNCH10PEN | GPIO_PRS_ROUTEEN_ASYNCH11PEN;

  // *********************************************************************
  // Подаём сигнал 2 и 3 канала на входы CC0 и CC1 таймеров 2 и 4
  // *********************************************************************

  PRS->CONSUMER_TIMER2_CC0 = 2;
  PRS->CONSUMER_TIMER4_CC0 = 2;
#ifndef SMOOTH
  PRS->CONSUMER_TIMER2_CC1 = 3;
  PRS->CONSUMER_TIMER4_CC1 = 3;
#endif

// Запускаем в работу.
  TIMER2->CMD = TIMER_CMD_START;
#ifndef SMOOTH
  TIMER4->CMD = TIMER_CMD_START;
#endif
  TIMER1->CMD = TIMER_CMD_START;
}

#ifndef SMOOTH
void show_left_scale(unsigned int element_count) {
  TIMER2->CC[2].OCB = element_count;
}

void show_right_scale(unsigned int element_count) {
  TIMER4->CC[2].OCB = element_count;
}
#else
void show_left_scale(unsigned int element_count) {
  if (element_count >= ch_durations_last) {
      element_count =  ch_durations_last - 1;
  }
  TIMER2->CC[1].OCB = (element_count) ? channel_durations[element_count-1] - 10 : 0;
}

void show_right_scale(unsigned int element_count) {
  if (element_count >= ch_durations_last) {
      element_count =  ch_durations_last - 1;
  }
  TIMER2->CC[2].OCB = (element_count) ? channel_durations[element_count-1] - 10 : 0;
}

#endif

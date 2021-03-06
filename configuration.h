/*
 * configuration.h
 *
 *  Created on: 3 июн. 2021 г.
 *      Author: Владимир Бирюков.
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

//#define PREV_VERSION

// Определение, где находится кнопка переводящая устройство
// в режим конфигурации.
#define KEY_PORT  gpioPortC
#define KEY_PIN   7

// Светодиоды, для индикации разных режимов
// Зажигается, если устройство доступно для конфигурирования
#define LED_ACCESS_PORT  gpioPortC
#define LED_ACCESS_PIN   6
// Зажигается, если устройство имеет соединение с конфигуратором
#define LED_CONNECT_PORT  gpioPortC
#define LED_CONNECT_PIN   5

#define LED_FN1_PORT  gpioPortC
#define LED_FN1_PIN   4
#define LED_FN2_PORT  gpioPortC
#define LED_FN2_PIN   3
#define LED_FN3_PORT  gpioPortC
#define LED_FN3_PIN   2
#define LED_FN4_PORT  gpioPortA
#define LED_FN4_PIN   4

#ifdef PREV_VERSION
#undef  KEY_PIN
#define KEY_PIN   6
#undef  LED_ACCESS_PIN
#define LED_ACCESS_PIN   7
#endif

// Цена одного элемента шкалы пол-градуса или градус
// откомментировать одну строчку
//#define DEGREE
#define HALFDEGREE

// Определение какой элемент шкалы является нулём градусов цельсия.
// 91 - для BBG12201, 51 - для ИН-34.
// сдвиг на 1 из-за того что катод сброса - сегмент 1, а не 0.
// Число должно быть нечетной десяткой, иначе надо или отказаться от
// выделения яркостью каждого 10 отсчета (DISABLE_HILIGHT),
// или переделать таблицу sequence в файле common/Timer_scan.c,
// что в общем случае правильно при применении другого индикатора.
#define T_SCALE_OFFSET  91
#define RH_SCALE_OFFSET  1

// Определить если индикатор имеет 103 отображающих элемента
//#define IN34

// Если применяется шкала с выделением каждого 5-го элемента, как ИН-34-1, то
// выключить выделение яркостью
//#define DISABLE_HILIGHT

// Конфигурация Screensaver'а -
// продолжительность и интервал в секундах (очень приблизительно)
#define SCREENSAVER_LENGTH  12
#define SCREENSAVER_INTERVAL  600



#endif /* CONFIGURATION_H_ */

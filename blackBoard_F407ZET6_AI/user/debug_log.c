/**
  ******************************************************************************
  * @file    debug_log.c
  * @author  Fahad Mirza (fahadmirza8@gmail.com)
  * @brief   This file provides TensorFLow DebugLog() implementation
  ******************************************************************************
  *                             /\     /\
  *                            {  `---'  }
  *                            {  O   O  }
  *                            ~~>  V  <~~
  *                             \  \|/  /
  *                              `-----'____
  *                              /     \    \_
  *                             {       }\  )_\_   _
  *                             |  \_/  |/ /  \_\_/ )
  *                              \__/  /(_/     \__/
  *                                (__/
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tensorflow/lite/micro/debug_log.h"
//#include "stm32f7xx_hal.h"
//#include "stm32f7xx_hal_uart.h"
#include <stdio.h>
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* Extern Variables ---------------------------------------------------------*/
//extern UART_HandleTypeDef DebugUartHandler; // Defined in main.cpp


/* Function Definitions -----------------------------------------------------*/
//int __io_putchar(int ch)
//{
//	HAL_UART_Transmit(&DebugUartHandler, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	//CDC_Transmit_FS((uint8_t *)&ch, 1);
    //return ch;
//}

//int _write(int file, char *ptr, int len) {
//    // Выполняем передачу через USB
//    uint8_t result = CDC_Transmit_FS((uint8_t *)ptr, len);
//
//    // Проверяем состояние передачи (USBD_OK = 0)
//    if (result == USBD_OK) {
//        return len;  // Возвращаем успешный результат
//    } else {
//        // Возвратим -1 в случае ошибки
//        return -1;
//    }
//}

// Used by TFLite error_reporter
void DebugLog(const char *s)
{
	//fprintf(stderr, "%s", s);
	// NOTE: fprintf uses _write(), which is defined in syscall.c
	//       _write() uses __io_putchar(), which is a weak function
	//       and needs to be implemented by user.
}

void DebugLogFormatted(const char *format, ...) {
    char buffer[2048];  // Временный буфер для строки

    va_list args;  // Список аргументов
    va_start(args, format);  // Начало обработки аргументов
    vsnprintf(buffer, sizeof(buffer), format, args);  // Формирование строки
    va_end(args);  // Конец обработки аргументов

    uint16_t len = strlen(buffer);  // Вычисляем длину строки
    CDC_Transmit_FS((uint8_t *)buffer, len);  // Передаём строку напрямую в USB CDC
}


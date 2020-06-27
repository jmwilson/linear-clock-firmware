#ifndef HAL_SERIAL_PRINT_H
#define HAL_SERIAL_PRINT_H

#include <cinttypes>
#include "stm32g0xx_hal.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class HAL_Serial_Print {
public:
  HAL_Serial_Print();
  HAL_Serial_Print(UART_HandleTypeDef &uart);

  size_t write(const char *buffer);
  size_t write(const char *buffer, size_t size);
  size_t write(const uint8_t *buffer, size_t size);

  size_t print(const char[]);
  size_t print(char);
  size_t print(unsigned char, int = DEC);
  size_t print(int, int = DEC);
  size_t print(unsigned int, int = DEC);
  size_t print(long, int = DEC);
  size_t print(unsigned long, int = DEC);
  size_t print(double, int = 2);

  size_t println(const char[]);
  size_t println(char);
  size_t println(unsigned char, int = DEC);
  size_t println(int, int = DEC);
  size_t println(unsigned int, int = DEC);
  size_t println(long, int = DEC);
  size_t println(unsigned long, int = DEC);
  size_t println(double, int = 2);
  size_t println(void);

private:
  UART_HandleTypeDef *_uart;

  size_t printNumber(unsigned long n, uint8_t base);
  size_t printFloat(double, uint8_t);
};

#endif

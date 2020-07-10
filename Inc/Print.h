/*
  Print.h - Base class that provides print() and println()
  Copyright (c) 2008 David A. Mellis.  All right reserved.
  Copyright (c) 2020 James Wilson. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __PRINT_H
#define __PRINT_H

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>

#include "main.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print
{
public:
  Print(UART_HandleTypeDef &uart) : _uart(&uart) {}

  size_t write(const char *str) const {
    if (str == NULL) return 0;
    return write((const uint8_t *)str, strlen(str));
  }
  size_t write(const uint8_t *buffer, size_t size) const;
  size_t write(const char *buffer, size_t size) const {
    return write((const uint8_t *)buffer, size);
  }

  size_t print(const std::string &) const;
  size_t print(const char*) const;
  size_t print(char) const;
  size_t print(unsigned char, int = DEC) const;
  size_t print(int, int = DEC) const;
  size_t print(unsigned int, int = DEC) const;
  size_t print(long, int = DEC) const;
  size_t print(unsigned long, int = DEC) const;
  size_t print(double, int = 2) const;

  size_t println(const std::string &s) const;
  size_t println(const char*) const;
  size_t println(char) const;
  size_t println(unsigned char, int = DEC) const;
  size_t println(int, int = DEC) const;
  size_t println(unsigned int, int = DEC) const;
  size_t println(long, int = DEC) const;
  size_t println(unsigned long, int = DEC) const;
  size_t println(double, int = 2) const;
  size_t println(void) const;

private:
  UART_HandleTypeDef *_uart;
  size_t printNumber(unsigned long, uint8_t) const;
  size_t printFloat(double, uint8_t) const;
};

#endif // __PRINT_H
/**
 * Copyright 2020 James Wilson <jmw@jmw.name>. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "main.h"
#include "tlc5926.h"

static void TLC5926_ModeSwitch(bool special) {
  const uint32_t bsrr_frames[] {
    TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin << 16 | TLC592x_OE_Pin,
    TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin << 16 | TLC592x_OE_Pin << 16,
    TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin << 16 | TLC592x_OE_Pin,
    special
      ? TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin | TLC592x_OE_Pin
      : TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin << 16 | TLC592x_OE_Pin,
    TLC592x_CLK_Pin << 16 | TLC592x_LE_Pin << 16 | TLC592x_OE_Pin,
  };
  for (const auto frame: bsrr_frames) {
    // Clock low
    TLC592x_CLK_GPIO_Port->BSRR = frame;
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    // Clock high
    TLC592x_CLK_GPIO_Port->BSRR = TLC592x_CLK_Pin;
  }
  TLC592x_CLK_GPIO_Port->BSRR = TLC592x_CLK_Pin << 16;
}

void TLC5926_SwitchToSpecialMode(void)
{
  assert_param(TLC592x_CLK_GPIO_Port == TLC592x_OE_GPIO_Port
    && TLC592x_OE_GPIO_Port == TLC592x_LE_GPIO_Port);

  // Reconfigute CLK and OE pins as GPIO
  GPIO_InitTypeDef GPIO_InitStruct {};
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin|TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TLC592x_CLK_GPIO_Port, &GPIO_InitStruct);

  // Bit-bang the mode-switching sequence
  TLC5926_ModeSwitch(true);

  // Restore CLK to peripheral, but leave OE in GPIO
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(TLC592x_CLK_GPIO_Port, &GPIO_InitStruct);
}

void TLC5926_SwitchToNormalMode(void)
{
  assert_param(TLC592x_CLK_GPIO_Port == TLC592x_OE_GPIO_Port
    && TLC592x_OE_GPIO_Port == TLC592x_LE_GPIO_Port);

  // Reconfigute CLK and OE pins as GPIO
  GPIO_InitTypeDef GPIO_InitStruct {};
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin|TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TLC592x_CLK_GPIO_Port, &GPIO_InitStruct);

  // Bit-bang the mode-switching sequence
  TLC5926_ModeSwitch(false);

  // Restore CLK and OE pins to peripherals
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(TLC592x_CLK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;
  HAL_GPIO_Init(TLC592x_OE_GPIO_Port, &GPIO_InitStruct);
}

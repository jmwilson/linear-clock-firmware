#include <stdbool.h>

#include "main.h"
#include "tlc592x.h"

static void TLC592x_Mode_Switch(bool special) {
  uint32_t bsrr_frames[] = {
    (TLC592x_CLK_Pin << 16) | (TLC592x_LE_Pin << 16) | TLC592x_OE_Pin,
    (TLC592x_CLK_Pin << 16) | (TLC592x_LE_Pin << 16) | (TLC592x_OE_Pin << 16),
    (TLC592x_CLK_Pin << 16) | (TLC592x_LE_Pin << 16) | TLC592x_OE_Pin,
    special
      ? (TLC592x_CLK_Pin << 16) | TLC592x_LE_Pin | TLC592x_OE_Pin
      : (TLC592x_CLK_Pin << 16) | (TLC592x_LE_Pin << 16) | TLC592x_OE_Pin,
    (TLC592x_CLK_Pin << 16) | (TLC592x_LE_Pin << 16) | TLC592x_OE_Pin,
  };
  for (int8_t i = 0; i < sizeof(bsrr_frames)/sizeof(uint32_t); i++) {
    // Clock low
    GPIOA->BSRR = bsrr_frames[i];
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    // Clock high
    GPIOA->BSRR = TLC592x_CLK_Pin;
  }
  GPIOA->BSRR = TLC592x_CLK_Pin << 16;
}

void TLC592x_Switch_To_Special_Mode(void)
{
  // Reconfigute CLK and OE pins as GPIO
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin|TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Bit-bang the mode-switching sequence
  TLC592x_Mode_Switch(true);

  // Restore CLK to peripheral, but leave OE in GPIO
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TLC592x_Switch_To_Normal_Mode(void)
{
  // Reconfigute CLK and OE pins as GPIO
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin|TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Bit-bang the mode-switching sequence
  TLC592x_Mode_Switch(false);

  // Restore CLK and OE pins to peripherals
  GPIO_InitStruct.Pin = TLC592x_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TLC592x_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;
  HAL_GPIO_Init(TLC592x_OE_GPIO_Port, &GPIO_InitStruct);
}

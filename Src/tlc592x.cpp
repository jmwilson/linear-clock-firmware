#include "main.h"
#include "tlc592x.h"

extern TIM_HandleTypeDef htim14;  // CLK
extern TIM_HandleTypeDef htim16;  // OE
extern TIM_HandleTypeDef htim17;  // LE

TLC592x::TLC592x(void)
{
    state = TLC592x_STATE_READY;
    op = TLC592x_OP_SHIFT_OUT;
    txCount = 0;
}

void TLC592x::shiftOut(uint8_t *buffer, uint16_t size_in_bits)
{
    // Check if peripheral is busy

    // Mark busy
    state = TLC592x_STATE_BUSY;
    op = TLC592x_OP_SHIFT_OUT;
    txBuffer = buffer;
    txSize = size_in_bits;
    txCount = 0;

    // Set first bit out
    if (*buffer & 1) {
        HAL_GPIO_WritePin(TLC592x_SDO_GPIO_Port, TLC592x_SDO_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(TLC592x_SDO_GPIO_Port, TLC592x_SDO_Pin, GPIO_PIN_RESET);
    }

    // Start the clock
    TIM14->CNT = 0;
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim14);
}

void TLC592x::switchToSpecialMode()
{
    // Mark busy
    state = TLC592x_STATE_BUSY;
    op = TLC592x_OP_SWITCH_TO_SPECIAL;
    txSize = 5;
    txCount = 0;

    disablePWM();

    // Start the mode switching sequence: OE=1, LE=0
    HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);

    // Start the clock
    TIM14->CNT = 0;
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim14);
}

void TLC592x::switchToNormalMode()
{
    // Mark busy
    state = TLC592x_STATE_BUSY;
    op = TLC592x_OP_SWITCH_TO_NORMAL;
    txSize = 5;
    txCount = 0;

    disablePWM();

    // Start the mode switching sequence: OE=1, LE=0
    HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);

    // Start the clock
    TIM14->CNT = 0;
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim14);
}

void TLC592x::enablePWM(uint16_t count)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = count;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TLC592x_OE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;
    HAL_GPIO_Init(TLC592x_OE_GPIO_Port, &GPIO_InitStruct);

    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void TLC592x::disablePWM(void)
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TLC592x_OE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TLC592x_OE_GPIO_Port, &GPIO_InitStruct);
}

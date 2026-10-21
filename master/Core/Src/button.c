#include "button.h"
#include "lcd.h"
/* UART handles coming from main.c */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* last button state */

void Button_Task(void)

{
    static uint32_t lastTick = 0;
    char tx;
    if(HAL_GetTick() - lastTick < 100) return;
    lastTick = HAL_GetTick();
#if 0
    /* PB10 */
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
    {
        tx = 'A';
        HAL_UART_Transmit(&huart1,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart2,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart3,(uint8_t*)&tx,1,10);
    }

    /* PB11 */
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        tx = 'B';
        HAL_UART_Transmit(&huart1,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart2,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart3,(uint8_t*)&tx,1,10);
    }

    /* PB12 */

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    {
        tx = 'C';
        HAL_UART_Transmit(&huart1,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart2,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart3,(uint8_t*)&tx,1,10);
    }

    /* PB13 */

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        tx = 'D';
        HAL_UART_Transmit(&huart1,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart2,(uint8_t*)&tx,1,10);
        HAL_UART_Transmit(&huart3,(uint8_t*)&tx,1,10);
    }
#endif
}



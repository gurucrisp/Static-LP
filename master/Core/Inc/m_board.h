/**
  * @file    m_board.h
  * @brief   Board facts for the DISP SS3S LCD4 M1 master (STM32G070RBT6), taken from the existing master project. Only the hardware glue includes this.
  *          The keys are active LOW (pressed = pin reads RESET), as in the old firmware (raw_pressed). Their order on the board, left to right: MENU, START, STOP, LOG.
  *          The old firmware opens Set-Mode with PB10 + PB13 held together: those are the outer keys, MENU and LOG. IF MENU AND LOG ARE THE OTHER WAY ROUND ON YOUR BOARD (a short LOG press then
  *          turns the page and a short MENU press opens the log), swap the two defines below.
  */
#ifndef M_BOARD_H
#define M_BOARD_H
#include "main.h"

#define MKEY_MENU_PORT    GPIOB
#define MKEY_MENU_PIN     GPIO_PIN_10
#define MKEY_START_PORT   GPIOB
#define MKEY_START_PIN    GPIO_PIN_11
#define MKEY_STOP_PORT    GPIOB
#define MKEY_STOP_PIN     GPIO_PIN_12
#define MKEY_LOG_PORT     GPIOB
#define MKEY_LOG_PIN      GPIO_PIN_13
/* the OVERHEAT input: active LOW (the pin is pulled LOW when the heat sink / transformer thermostat says too hot); the board has an EXTERNAL pull-up */
#define MHEAT_PORT        GPIOA
#define MHEAT_PIN         GPIO_PIN_5

#endif
